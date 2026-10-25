// NativeDwc3 -- SM8150 USB0 (DWC3) bring-up, staged so a hang localises itself.
//
//  0  RPMh vote: ldoa5 (vdda-pll), ldoa2 (vdda33), ldoa12 (vdda18).
//  1  GCC: usb30_prim GDSC, SEC clkref, branch clocks, USB30_PRIM reset.
//  2  7nm femto HS PHY init (UTMI clock to the core).
//  3  read-only core identification (GSNPSID/GHWPARAMS).
//  4  (nabu-usb-init=1) core soft reset, device mode, RUN_STOP.
//
// Entry point is native_dwc3_bringup(), called by the UFS probe: the
// IOService personality cannot win the single IOPlatformDevice nub.
//
// References: dwc3/core.c, dwc3-qcom.c, phy-qcom-snps-femto-v2.c,
// gcc-sm8150.c, cmd-db.c, rpmh-rsc.c, qcom-rpmh-regulator.c.

#include "NativeDwc3.h"
#include "Dwc3Registers.h"
#include "GccUsb.h"

#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <kern/task.h>
#include <libkern/c++/OSSharedPtr.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>

extern "C" void nabu_mark_text(const char *s, unsigned int y);
extern "C" void nabu_mark_num (const char *label, unsigned long v, unsigned int y);
/* The host reads the UFS log window, so boot readings must go to the ring too:
 * IOLog no longer reaches it once the console leaves printbuf. */
extern "C" void nabu_log_puts(const char *s);
extern "C" void nabu_log_hex(const char *label, unsigned long long value);
/* Mid-window snapshot: a run that wedges past this point still leaves the
 * enumeration state on the medium from the boot thread, the same path the boot
 * probe already uses successfully. */
extern "C" void nabu_log_flush(int is_panic);
extern "C" int nabu_log_flush_tail(void);
/* Milestone probe: publish the ring synchronously so the window itself names the
 * last stage the bring-up reached. The previous runs could not tell a wedged
 * bring-up from a dead log thread because every write after the baseline was
 * silent; here each stage lands on the medium before the next register touch. */
extern "C" int nabu_log_flush_nowait(void);
static void logProbe(const char *tag)
{
  nabu_log_puts(tag);
  if (!nabu_log_flush_nowait()) {
    nabu_log_puts("p:flush-fail\n");
  }
}

/* Read by the periodic log writer in NativeUfs: 1 while the bring-up may still be
 * inside this file, and a counter that moves once per enumeration round. A window
 * open with a counter that stopped moving is a wedged controller, and the writer
 * resets the board rather than waiting for a power press nobody is there to give. */
extern "C" {
volatile unsigned int gDwc3Window = 0;
volatile unsigned int gDwc3Ticks = 0;
/* Ticks move even when the controller is wedged -- the poll loop keeps spinning --
 * so the watchdog in NativeUfs needs the event count instead: an event is progress,
 * a spin is not. gDwc3Setups separates "host never talked to us" from "host talked
 * and then we stopped answering". */
volatile unsigned int gDwc3Events = 0;
volatile unsigned int gDwc3Setups = 0;
/* Set by the watchdog just before it resets the board, cleared by the log
 * writer once it has taken the snapshot. This is the only way the ring gets
 * published while the DWC3 owns UFS. */
volatile unsigned int gFlushRequested = 0;
}

#define super IOService
OSDefineMetaClassAndStructors(NativeDwc3, IOService)

namespace {

OSSharedPtr<IOMemoryMap> g_keep;
OSSharedPtr<IOMemoryMap> g_gccKeep;
volatile uint32_t *g_regs = nullptr;
volatile uint32_t *g_gcc = nullptr;
int g_bringup_done = 0;

inline uint32_t rd(unsigned long off) { return g_regs[off / 4]; }
inline void wr(unsigned long off, uint32_t v) { g_regs[off / 4] = v; }
inline uint32_t grd(unsigned long off) { return g_gcc[off / 4]; }
inline void gwr(unsigned long off, uint32_t v) { g_gcc[off / 4] = v; }

static uintptr_t mapPhys(uint64_t phys, uint64_t length, OSSharedPtr<IOMemoryMap> &keep)
{
  IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
  if (!memory) { return 0; }
  keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
  memory->release();
  return keep ? static_cast<uintptr_t>(keep->getVirtualAddress()) : 0;
}

static bool waitMask(volatile uint32_t *reg, uint32_t mask, uint32_t value, unsigned spins)
{
  for (unsigned i = 0; i < spins; ++i) {
    if ((*reg & mask) == value) { return true; }
    __asm__ volatile("yield" ::: "memory");
  }
  return false;
}

// ---- RPMh / cmd_db --------------------------------------------------------------
#define CMD_DB_BASE 0x85F20000ULL
#define CMD_DB_SIZE 0x20000ULL
#define RSC_DRV2_BASE 0x18220000ULL
#define RSC_DRV_SIZE 0x10000ULL
#define RSC_TCS_OFFSET 0xd00
#define RSC_TCS_CONTROL 0x14
#define RSC_TCS_CMD_ENABLE 0x1c
#define RSC_TCS_CMD_MSGID 0x30
#define RSC_TCS_CMD_ADDR 0x34
#define RSC_TCS_CMD_DATA 0x38
#define RSC_TCS_CMD_STATUS 0x3c
#define TCS_AMC_ENABLE (1u << 16)
#define TCS_AMC_TRIGGER (1u << 24)
#define RPMH_MSGID_LEN 8u
#define RPMH_MSGID_RESP (1u << 8)
#define RPMH_MSGID_WRITE (1u << 16)
#define RPMH_CMD_COMPL (1u << 16)

static OSSharedPtr<IOMemoryMap> g_cmdKeep, g_rscKeep;
static volatile uint8_t *g_cmd;
static volatile uint32_t *g_rsc;

static uint16_t cmdRd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t cmdRd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint32_t rscRd(unsigned long off) { return g_rsc[off / 4]; }
static void rscWr(unsigned long off, uint32_t v) { g_rsc[off / 4] = v; }

static uint32_t rpmhCmdDbAddr(const char *name)
{
  static const uint8_t magic[4] = { 0xdb, 0x30, 0x03, 0x0c };
  if (g_cmd == 0) {
    return 0;
  }
  for (unsigned i = 0; i < 4; ++i) {
    if (g_cmd[4 + i] != magic[i]) {
      nabu_log_puts("rpmh:cmd-db-magic-bad\n");
      return 0;
    }
  }
  unsigned len = 0;
  while (name[len] != 0 && len < 8) {
    ++len;
  }
  for (unsigned r = 0; r < 8; ++r) {
    const uint8_t *rh = const_cast<const uint8_t *>(g_cmd) + 8 + r * 16;
    if (cmdRd16(rh + 0) == 0) {
      break;
    }
    uint16_t hoff = cmdRd16(rh + 2), cnt = cmdRd16(rh + 6);
    const uint8_t *ent = const_cast<const uint8_t *>(g_cmd) + 144 + hoff;
    for (unsigned e = 0; e < cnt; ++e, ent += 24) {
      bool match = true;
      for (unsigned c = 0; c < 8 && match; ++c) {
        uint8_t want = (c < len) ? static_cast<uint8_t>(name[c]) : 0;
        if (ent[c] != want) {
          match = false;
        }
      }
      if (match) {
        return cmdRd32(ent + 16);
      }
    }
  }
  return 0;
}

static bool rpmhWrite(uint32_t addr, uint32_t data)
{
  const unsigned long tcs = RSC_TCS_OFFSET; // active TCS 0
  rscWr(tcs + RSC_TCS_CMD_MSGID, RPMH_MSGID_LEN | RPMH_MSGID_RESP | RPMH_MSGID_WRITE);
  rscWr(tcs + RSC_TCS_CMD_ADDR, addr);
  rscWr(tcs + RSC_TCS_CMD_DATA, data);
  rscWr(tcs + RSC_TCS_CMD_ENABLE, 1);

  uint32_t ctrl = rscRd(tcs + RSC_TCS_CONTROL);
  ctrl &= ~TCS_AMC_TRIGGER;
  rscWr(tcs + RSC_TCS_CONTROL, ctrl);
  ctrl &= ~TCS_AMC_ENABLE;
  rscWr(tcs + RSC_TCS_CONTROL, ctrl);
  rscWr(tcs + RSC_TCS_CONTROL, ctrl | TCS_AMC_ENABLE);
  rscWr(tcs + RSC_TCS_CONTROL, ctrl | TCS_AMC_ENABLE | TCS_AMC_TRIGGER);

  for (unsigned i = 0; i < 1000000; ++i) {
    if (rscRd(tcs + RSC_TCS_CMD_STATUS) & RPMH_CMD_COMPL) {
      nabu_log_hex("rpmh:sts=", rscRd(tcs + RSC_TCS_CMD_STATUS));
      nabu_log_hex("rpmh:resp=", rscRd(tcs + 0x40));
      return true;
    }
    __asm__ volatile("yield" ::: "memory");
  }
  nabu_log_hex("rpmh:sts-timeout=", rscRd(tcs + RSC_TCS_CMD_STATUS));
  return false;
}

static bool rpmhRead(uint32_t addr, uint32_t *out)
{
  const unsigned long tcs = RSC_TCS_OFFSET;
  rscWr(tcs + RSC_TCS_CMD_MSGID, RPMH_MSGID_LEN | RPMH_MSGID_RESP);
  rscWr(tcs + RSC_TCS_CMD_ADDR, addr);
  rscWr(tcs + RSC_TCS_CMD_ENABLE, 1);
  uint32_t ctrl = rscRd(tcs + RSC_TCS_CONTROL);
  ctrl &= ~TCS_AMC_TRIGGER; rscWr(tcs + RSC_TCS_CONTROL, ctrl);
  ctrl &= ~TCS_AMC_ENABLE; rscWr(tcs + RSC_TCS_CONTROL, ctrl);
  rscWr(tcs + RSC_TCS_CONTROL, ctrl | TCS_AMC_ENABLE);
  rscWr(tcs + RSC_TCS_CONTROL, ctrl | TCS_AMC_ENABLE | TCS_AMC_TRIGGER);
  for (unsigned i = 0; i < 1000000; ++i) {
    if (rscRd(tcs + RSC_TCS_CMD_STATUS) & RPMH_CMD_COMPL) {
      *out = rscRd(tcs + 0x40);
      return true;
    }
    __asm__ volatile("yield" ::: "memory");
  }
  return false;
}

// Voltage register takes mV; mode 7 = PMIC5 LDO HPM; enable = 1.
static bool rpmhSetLdo(uint32_t base, uint32_t mV)
{
  if (base == 0) {
    return false;
  }
  bool ok = rpmhWrite(base + 0x0, mV);
  ok = rpmhWrite(base + 0x8, 7) && ok;
  ok = rpmhWrite(base + 0x4, 1) && ok;
  return ok;
}

static void rpmhProbe(void)
{
  g_cmd = reinterpret_cast<volatile uint8_t *>(mapPhys(CMD_DB_BASE, CMD_DB_SIZE, g_cmdKeep));
  g_rsc = reinterpret_cast<volatile uint32_t *>(mapPhys(RSC_DRV2_BASE, RSC_DRV_SIZE, g_rscKeep));
  if (g_cmd == 0 || g_rsc == 0) {
    nabu_log_puts("rpmh:map-bad\n");
    return;
  }
  nabu_log_hex("rpmh:ldoa2=", rpmhCmdDbAddr("ldoa2"));
  // The PHY's ref_clk_src is RPMH_CXO_CLK; if that clock is an RPMh resource it
  // has to be voted on, exactly like the rails. Look the names up first.
  nabu_log_hex("rpmh:cxo=", rpmhCmdDbAddr("cxo"));
  nabu_log_hex("rpmh:xo=", rpmhCmdDbAddr("xo"));
  nabu_log_hex("rpmh:clk=", rpmhCmdDbAddr("clk"));
  nabu_log_hex("rpmh:ldoa5=", rpmhCmdDbAddr("ldoa5"));
  nabu_log_hex("rpmh:ldoa12=", rpmhCmdDbAddr("ldoa12"));
  nabu_log_hex("rpmh:drvid=", rscRd(0));
  nabu_log_hex("rpmh:config=", rscRd(0x4));
  nabu_log_hex("rpmh:tcs_ctrl=", rscRd(RSC_TCS_OFFSET + RSC_TCS_CONTROL));
  nabu_log_hex("rpmh:tcs_en=", rscRd(RSC_TCS_OFFSET + RSC_TCS_CMD_ENABLE));
  nabu_log_hex("rpmh:tcs_stat=", rscRd(RSC_TCS_OFFSET + RSC_TCS_CMD_STATUS));
  nabu_log_hex("rpmh:ldob1=", rpmhCmdDbAddr("ldob1"));
  nabu_log_hex("rpmh:ldob2=", rpmhCmdDbAddr("ldob2"));
  nabu_log_hex("rpmh:ldob3=", rpmhCmdDbAddr("ldob3"));
  nabu_log_hex("rpmh:ldob4=", rpmhCmdDbAddr("ldob4"));
  nabu_log_hex("rpmh:chld=", rscRd(0x0c));
  // Linux writes the ACTIVE TCS mask here at probe (rpmh_probe_tcs_config tail).
  rscWr(RSC_TCS_OFFSET + 0x00, 0x3);
}

static bool rpmhUsbPhyRails(void)
{
  // USB core and PHY sit in the CX domain; without an active corner vote their
  // registers read 0 while GCC (always-on domain) works. Corner 3 = HIGH.
  // The femto PHY clocks from RPMH_CXO_CLK (clk-rpmh: xo.lvl, on = 0x3).
  // Without this vote the PHY AHB/PLL have no reference and read back 0.
  const uint32_t xo = rpmhCmdDbAddr("xo.lvl");
  nabu_log_hex("rpmh:xo=", xo);
  if (xo != 0) { (void)rpmhWrite(xo, 0x3); }
  const uint32_t cx = rpmhCmdDbAddr("cx.lvl");
  nabu_log_hex("rpmh:cx=", cx);
  // MX (DDR) must not be voted from here: corner 5 reset the board on 2026-10-24.
  if (cx != 0) { (void)rpmhWrite(cx, 5); }
  const uint32_t l2 = rpmhCmdDbAddr("ldoa2");
  const uint32_t l5 = rpmhCmdDbAddr("ldoa5");
  const uint32_t l12 = rpmhCmdDbAddr("ldoa12");
  bool ok = true;
  ok = rpmhSetLdo(l5, 880) && ok;   // vdda-pll
  ok = rpmhSetLdo(l2, 3072) && ok;  // vdda33
  ok = rpmhSetLdo(l12, 1800) && ok; // vdda18
  uint32_t v = 0;
  if (rpmhRead(l5 + 0x0, &v)) { nabu_log_hex("rpmh:l5v=", v); }
  if (rpmhRead(l5 + 0x4, &v)) { nabu_log_hex("rpmh:l5e=", v); }
  if (rpmhRead(l2 + 0x4, &v)) { nabu_log_hex("rpmh:l2e=", v); }
  if (rpmhRead(l2 + 0x0, &v)) { nabu_log_hex("rpmh:l2v=", v); }
  if (rpmhRead(l12 + 0x0, &v)) { nabu_log_hex("rpmh:l12v=", v); }
  if (rpmhRead(xo, &v)) { nabu_log_hex("rpmh:xov=", v); }
  nabu_log_puts(ok ? "rpmh:rails-ok\n" : "rpmh:rails-fail\n");
  return ok;
}

// ---- GCC USB30_PRIM -------------------------------------------------------------
static unsigned gccUsbPowerOn(void)
{
  const uintptr_t va = mapPhys(GCC_BASE, 0x100000, g_gccKeep);
  g_gcc = reinterpret_cast<volatile uint32_t *>(va);
  if (g_gcc == nullptr) {
    return 0xff;
  }

  uint32_t gdscr = grd(GCC_USB30_PRIM_GDSCR);
  nabu_log_hex("dwc3:gdscr=", gdscr);
  gwr(GCC_USB30_PRIM_GDSCR, gdscr & ~GCC_GDSCR_SW_COLLAPSE);
  if (!waitMask(g_gcc + GCC_USB30_PRIM_GDSCR / 4, GCC_GDSCR_PWR_ON, GCC_GDSCR_PWR_ON, 100000)) {
    nabu_log_puts("dwc3:gdsc-timeout\n");
    return 0xfe;
  }

  // The HS PHY's "ref_clk_src" is a USB3 clkref branch. Every other clock this
  // controller uses is USB30_PRIM_*, so enabling only the SEC branch (as the
  // earlier code did) left the PHY PLL without a reference. Drive both and log
  // the halt bits: bit31 set means the branch is not running.
  gwr(GCC_USB3_PRIM_CLKREF_CLK, grd(GCC_USB3_PRIM_CLKREF_CLK) | GCC_BRANCH_ENABLE_BIT);
  gwr(GCC_USB3_SEC_CLKREF_CLK, grd(GCC_USB3_SEC_CLKREF_CLK) | GCC_BRANCH_ENABLE_BIT);
  for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  nabu_log_hex("gcc:refprim=", grd(GCC_USB3_PRIM_CLKREF_CLK));
  nabu_log_hex("gcc:refsec=", grd(GCC_USB3_SEC_CLKREF_CLK));

  static const unsigned long branches[] = {
    GCC_USB30_PRIM_MASTER_CLK,
    GCC_USB30_PRIM_SLEEP_CLK,
    GCC_USB30_PRIM_MOCK_UTMI_CLK,
    GCC_USB3_PRIM_PHY_AUX_CLK,
    GCC_USB3_PRIM_PHY_COM_AUX_CLK,
    GCC_USB3_PRIM_PHY_PIPE_CLK,
    GCC_AGGRE_USB3_PRIM_AXI_CLK,
    GCC_CFG_NOC_USB3_PRIM_AXI_CLK,
  };
  for (unsigned i = 0; i < sizeof(branches) / sizeof(branches[0]); ++i) {
    gwr(branches[i], grd(branches[i]) | GCC_BRANCH_ENABLE_BIT);
  }
  unsigned failed = 0;
  for (unsigned i = 0; i < sizeof(branches) / sizeof(branches[0]); ++i) {
    if (!waitMask(g_gcc + branches[i] / 4, GCC_BRANCH_CLK_OFF, 0, 100000)) {
      ++failed;
    }
  }

  // qcom BCRs do not self-clear: assert with 1, then write 0 to release.
  // Leaving them at 1 keeps the PHY and the core in reset (reads return 0).
  gwr(GCC_USB_PHY_CFG_AHB2PHY_BCR, 0x1);
  for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  gwr(GCC_USB_PHY_CFG_AHB2PHY_BCR, 0x0);
  // resets = <&gcc GCC_USB30_PRIM_BCR> in the glue node: assert then release.
  gwr(GCC_USB30_PRIM_BCR, 0x1);
  for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  gwr(GCC_USB30_PRIM_BCR, 0x0);
  return failed;
}

// ---- 7nm femto HS PHY -----------------------------------------------------------
#define QUSB2_BASE 0x088E2000ULL
#define FEMTO_UTMI_CTRL0 0x3c
#define FEMTO_UTMI_CTRL1 0x40
#define FEMTO_UTMI_CTRL5 0x50
#define FEMTO_COMMON0 0x54
#define FEMTO_COMMON1 0x58
#define FEMTO_COMMON2 0x5c
#define FEMTO_CTRL1 0x60
#define FEMTO_CTRL2 0x64
#define FEMTO_CFG0 0x94
#define FEMTO_REFCLK 0xa0
// This board binds drivers/usb/phy/phy-msm-snps-hs.c ("msm-usb-hsphy"), not
// qusb2: 0xb4 is RTUNE_SEL here, not a port-powerdown register.
#define FEMTO_RTUNE_SEL 0xb4

// QSCRATCH block inside the dwc3-mms wrapper (drivers/usb/dwc3/dwc3-msm.c).
#define QSCRATCH_REG_OFFSET 0x000F8800UL
#define QSCRATCH_GENERAL_CFG (QSCRATCH_REG_OFFSET + 0x08)
#define QSCRATCH_HS_PHY_CTRL (QSCRATCH_REG_OFFSET + 0x10)
#define QSCRATCH_CGCTL (QSCRATCH_REG_OFFSET + 0x28)
#define QSCRATCH_PIPE_UTMI_CLK_SEL (1u << 0)
#define QSCRATCH_PIPE3_PHYSTATUS_SW (1u << 3)
#define QSCRATCH_PIPE_UTMI_CLK_DIS (1u << 8)
#define QSCRATCH_UTMI_OTG_VBUS_VALID (1u << 20)

static OSSharedPtr<IOMemoryMap> g_phyKeep;
static volatile uint32_t *g_phy;

static uint32_t phyRd(unsigned long off) { return g_phy[off / 4]; }
static void phyWr(unsigned long off, uint32_t v) { g_phy[off / 4] = v; }
static void phyWrMask(unsigned long off, uint32_t mask, uint32_t val)
{
  uint32_t v = phyRd(off);
  phyWr(off, (v & ~mask) | (val & mask));
}

static int g_phyInitMode = 1;

static bool femtoPhyInit(void)
{
  const uintptr_t va = mapPhys(QUSB2_BASE, 0x1000, g_phyKeep);
  g_phy = reinterpret_cast<volatile uint32_t *>(va);
  if (g_phy == nullptr) {
    return false;
  }
  nabu_log_hex("phy:r0=", phyRd(0x0));
  nabu_log_hex("phy:common0i=", phyRd(FEMTO_COMMON0));
  nabu_log_hex("phy:common1i=", phyRd(FEMTO_COMMON1));
  nabu_log_hex("phy:ctrl2i=", phyRd(FEMTO_CTRL2));

  // msm_hsphy_init(), in the vendor driver's exact order.
  phyWrMask(FEMTO_CFG0, 1u << 1, 1u << 1);        // CMN_CTRL_OVERRIDE_EN
  phyWrMask(FEMTO_UTMI_CTRL5, 1u << 1, 1u << 1);  // POR
  phyWrMask(FEMTO_COMMON0, 0x70, 0);              // FSEL = 0
  phyWrMask(FEMTO_COMMON1, 1u << 5, 1u << 5);     // PLLBTUNE
  phyWrMask(FEMTO_REFCLK, 0x3, 0x2);              // REFCLK_SEL_DEFAULT
  phyWrMask(FEMTO_COMMON1, 1u << 4, 1u << 4);     // VBUSVLDEXTSEL0 (vendor default)
  phyWrMask(FEMTO_CTRL1, 1u << 0, 1u << 0);       // VBUSVLDEXT0     (vendor default)

  // qcom,param-override-seq = <4 0x70 0x2c 0x74>. hsusb_phy_write_seq writes
  // seq[i] TO base + seq[i+1], so the pairs are 0x70 <- 0x04 and 0x74 <- 0x2c
  // (the PARAMETER_OVERRIDE_X1/X2 registers), the opposite of the obvious read.
  phyWr(0x70, 0x04);
  phyWr(0x74, 0x2c);

  {
    // Use the external resistor only when the eFuse calibration is unprogrammed.
    OSSharedPtr<IOMemoryMap> rcalKeep;
    volatile uint32_t *rcal =
      reinterpret_cast<volatile uint32_t *>(mapPhys(0x007801F8ULL & ~0xFFFULL, 0x1000, rcalKeep));
    uint32_t rcalCode = 0;
    if (rcal != nullptr) {
      rcalCode = rcal[(0x1F8 & 0xFFF) / 4] & 0x01E00000u;
    }
    nabu_log_hex("phy:rcal=", rcalCode);
    if (rcalCode == 0) {
      phyWrMask(FEMTO_RTUNE_SEL, 1u << 0, 1u << 0);  // RTUNE_SEL
    }
  }

  phyWrMask(FEMTO_COMMON2, 1u << 0, 1u << 0);     // VREGBYPASS (vendor value)
  phyWrMask(FEMTO_CTRL2, (1u << 3) | (1u << 2), (1u << 3) | (1u << 2));
  phyWrMask(FEMTO_UTMI_CTRL0, 1u << 0, 1u << 0);  // SLEEPM
  phyWrMask(FEMTO_UTMI_CTRL5, 1u << 1, 0);        // POR = 0
  phyWrMask(FEMTO_CTRL2, 1u << 3, 0);             // SUSPEND_N_SEL = 0, SUSPEND_N stays 1
  phyWrMask(FEMTO_CFG0, 1u << 1, 0);              // CMN override off

  for (unsigned i = 0; i < 200000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  nabu_log_hex("phy:common0f=", phyRd(FEMTO_COMMON0));
  nabu_log_hex("phy:common1f=", phyRd(FEMTO_COMMON1));
  nabu_log_hex("phy:common2f=", phyRd(FEMTO_COMMON2));
  nabu_log_hex("phy:ctrl0f=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_hex("phy:ctrl1f=", phyRd(FEMTO_CTRL1));
  nabu_log_hex("phy:ctrl2f=", phyRd(FEMTO_CTRL2));
  nabu_log_hex("phy:ctrl5f=", phyRd(FEMTO_UTMI_CTRL5));
  nabu_log_hex("phy:cfg0f=", phyRd(FEMTO_CFG0));
  nabu_log_hex("phy:ref=", phyRd(FEMTO_REFCLK));
  nabu_log_hex("phy:x1=", phyRd(0x70));
  nabu_log_hex("phy:x2=", phyRd(0x74));
  nabu_log_hex("phy:rtune=", phyRd(FEMTO_RTUNE_SEL));
  // PLL block (0x00..0x38) in full: the vendor driver never reads a lock bit,
  // but if one exists it will be the bit that turns on after init.
  for (unsigned off = 0; off <= 0x38; off += 4) {
    nabu_log_hex("phy:pll=", (off << 16) | (phyRd(off) & 0xffu));
  }
  nabu_log_puts("phy:init-done\n");
  return true;
}

// msm_hsphy_drive_dp_pulse() (phy-msm-snps-hs.c). On CDP the charger's BC1.2
// unit has been sitting on these same D+/D- lines, so dwc3-msm makes the device
// re-announce itself by driving a FullSpeed J state and then releasing every
// override. dwc3_msm_vbus_notifier calls it *before* dwc3_otg_start_peripheral,
// i.e. before DCTL.RUN_STOP. The vendor's 200 ms has no usable time base this
// early, so it is a bounded yield loop like every other settle in this file.
static void femtoDpPulse(void)
{
  if (g_phy == nullptr) { return; }
  phyWrMask(FEMTO_CFG0, 3u, 3u);                  // CMN + DATAPATH override
  phyWrMask(FEMTO_UTMI_CTRL0, 0x18, 0x00);        // OPMODE = NORMAL
  phyWrMask(FEMTO_UTMI_CTRL0, 1u << 5, 1u << 5);  // TERMSEL
  phyWrMask(FEMTO_UTMI_CTRL1, 1u << 0, 1u << 0);  // XCVRSEL lives at 0x40
  nabu_log_hex("phy:dp-pre=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_hex("phy:dp-xcvr=", phyRd(FEMTO_UTMI_CTRL1));
  for (unsigned i = 0; i < 4000000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  phyWrMask(FEMTO_UTMI_CTRL0, 1u << 5, 0);        // TERMSEL off
  phyWrMask(FEMTO_UTMI_CTRL1, 1u << 0, 0);        // XCVRSEL off
  phyWrMask(FEMTO_CFG0, 3u, 0);                   // both overrides off
  nabu_log_hex("phy:dp-post=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_hex("phy:dp-cfg=", phyRd(FEMTO_CFG0));
  // 0x60 bit0 is VBUSVLDEXT0, not XCVRSEL: the pulse must leave it set.
  nabu_log_hex("phy:dp-vbus=", phyRd(FEMTO_CTRL1));
  nabu_log_hex("phy:dp-sel=", phyRd(FEMTO_COMMON1));
}

// ---- bring-up -------------------------------------------------------------------
// ---- USB event buffer / SMMU ----------------------------------------------------
// The DWC3 event ring must be physically contiguous and reachable by the controller's
// stream (0x140 on apps_smmu). The address floor is the same one UFS proved: buffers
// below 0xC8000000 reset the board on the first command.

extern "C" int native_smmu_bypass(uint16_t stream, uint64_t dma, uint64_t bytes);

static IOBufferMemoryDescriptor *g_evtDma;
static OSSharedPtr<IOMemoryMap> g_evtKeep;
static volatile uint32_t *g_evt;
static uint64_t g_evtPhys;
static uint32_t g_evtOff;      // software read cursor inside the event ring

static bool usbEventInit(void)
{
  nabu_log_puts("usb:alloc\n");
  g_evtDma = IOBufferMemoryDescriptor::inTaskWithOptions(
    kernel_task,
    static_cast<IOOptionBits>(kIODirectionInOut) |
      static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
      static_cast<IOOptionBits>(kIOMemoryMapperNone),
    4096, 4096);
  if (g_evtDma == nullptr) {
    nabu_log_puts("usb:dma-fail\n");
    return false;
  }
  nabu_log_puts("usb:alloc-done\n");
  IOByteCount contiguous = 0;
  g_evtPhys = g_evtDma->getPhysicalSegment(0, &contiguous, kIOMemoryMapperNone);
  // Read the ring through a mapping of the address the controller was given.
  // getBytesNoCopy() returns a different page here: run 28 showed the core's
  // events at 0xcaa47000 (0x301, 0x50301) while that virtual address read zero,
  // which is why every previous poll looked empty.
  g_evt = reinterpret_cast<volatile uint32_t *>(mapPhys(g_evtPhys, 4096, g_evtKeep));
  nabu_log_hex("usb:evtpa=", g_evtPhys);
  nabu_log_hex("usb:evtlen=", contiguous);
  if (g_evt == nullptr || g_evtPhys == 0 || (g_evtPhys & 4095) || contiguous < 4096) {
    nabu_log_puts("usb:dma-bad\n");
    return false;
  }
  for (unsigned i = 0; i < 1024; ++i) { g_evt[i] = 0; }
  nabu_log_puts("usb:smmu\n");
  // The USB event ring needs its own SMMU stream mapping, but allocating a fresh
  // context bank from inside NativeUfs::start can pick a bank UFS is about to
  // use (the scan trusts whatever SMR/S2CR state survived a hard reset), which
  // wedges UFS DMA: the kernel then sits in IOFindBSRoot with no panic and no
  // reset. Default is off; enable with nabu-usb-smmu=1 once UFS is known clean.
  int smmuMap = 0;
  (void)PE_parse_boot_argn("nabu-usb-smmu", &smmuMap, sizeof(smmuMap));
  nabu_log_hex("usb:smmu-en=", smmuMap);
  // Read-only: does the firmware already carry a stream entry for the USB SID?
  // If it does, the event ring can DMA through it and no context bank of ours is
  // needed; if it does not, enabling the ring would fault instead.
  {
    OSSharedPtr<IOMemoryMap> smmuKeep;
    volatile uint32_t *smmu =
      reinterpret_cast<volatile uint32_t *>(mapPhys(0x15000000ULL, 0x8000, smmuKeep));
    if (smmu == nullptr) {
      nabu_log_puts("smmu:map-fail\n");
    } else {
      nabu_log_hex("smmu:idr0=", smmu[0x20 / 4]);
      nabu_log_hex("smmu:gfsr=", smmu[0x48 / 4]);
      for (unsigned i = 0; i < 128; ++i) {
        uint32_t smr = smmu[(0x800 / 4) + i];
        if (!(smr & 0x80000000u)) { continue; }
        if ((smr & 0xffffu) != 0x140u) { continue; }
        nabu_log_hex("smmu:sid140smr=", smr);
        {
          // The event ring writes do not land in our buffer with the firmware's
          // S2CR (reads back 3, not the ARM bypass value 2) and GFSR stays 0, so
          // the address is being translated away. Flip ONLY this stream's TYPE
          // to bypass; no context bank is allocated. UFS has its own slot.
          const uint32_t old = smmu[(0xc00 / 4) + i];
          nabu_log_hex("smmu:s2cr-old=", old);
          if ((old & 3u) != 2u) {
            smmu[(0xc00 / 4) + i] = (old & ~3u) | 2u;
            __asm__ volatile("dsb sy" ::: "memory");
          }
          nabu_log_hex("smmu:s2cr-new=", smmu[(0xc00 / 4) + i]);
        }
        break;
      }
    }
  }
  if (smmuMap) {
    (void)native_smmu_bypass(0x140, g_evtPhys, 4096);
  }
  nabu_log_puts("usb:smmu-done\n");
  logProbe("p:evt\n");
  wr(DWC3_GEVNTADRLO(0), static_cast<uint32_t>(g_evtPhys));
  wr(DWC3_GEVNTADRHI(0), static_cast<uint32_t>(g_evtPhys >> 32));
  wr(DWC3_GEVNTSIZ(0), 4096);
  wr(DWC3_GEVNTCOUNT(0), 0);
  g_evtOff = 0;
  wr(DWC3_GEVTEN, 1);
  wr(DWC3_DEVTEN, 0xf); // disconnect, reset, connect-done, setup
  nabu_log_hex("usb:evtadr=", rd(DWC3_GEVNTADRLO(0)));
  nabu_log_hex("usb:evtadrhi=", rd(DWC3_GEVNTADRHI(0)));
  nabu_log_hex("usb:evtsiz=", rd(DWC3_GEVNTSIZ(0)));
  nabu_log_hex("usb:evtcnt=", rd(DWC3_GEVNTCOUNT(0)));
  return true;
}

// SPMI write through the v5 R/W channel window (chnls at 0x0C600000,
// offset 0x10000*apid). Returns false on timeout or a denied/failed status.
static bool spmiWrite(uint32_t apid, uint32_t addr, uint32_t val)
{
  OSSharedPtr<IOMemoryMap> keep;
  volatile uint32_t *chn = reinterpret_cast<volatile uint32_t *>(
    mapPhys(0x0C600000ULL + 0x10000ULL * apid, 0x1000, keep));
  if (chn == nullptr) { return false; }
  chn[0] = (14u << 27) | (2u << 20) | ((addr & 0xffu) << 4);   // WRITE, sid 2, 1 byte
  chn[4] = val;
  __asm__ volatile("dsb sy" ::: "memory");
  for (unsigned i = 0; i < 20000; ++i) {
    const uint32_t st = chn[2];
    if (st & 1u) { return (st & 6u) == 0; }
    __asm__ volatile("yield" ::: "memory");
  }
  return false;
}

// ---- control endpoint (EP0) + CDC-ACM -------------------------------------------
// The host has exactly one control transfer in flight at a time, so this is a
// synchronous state machine advanced out of the event ring: arm a SETUP TRB,
// parse the packet when it completes, answer it, then wait for the on-demand
// status phase. Everything lives in one 4 KiB block -- the single TRB slot at
// the bottom, the staging buffer after it -- because the core needs the TRB and
// its data buffer to be physically contiguous only within themselves, and one
// allocation keeps both on the same SMMU stream that UFS already proved.

static IOBufferMemoryDescriptor *g_ep0Dma;
static OSSharedPtr<IOMemoryMap> g_ep0Keep;
static volatile uint32_t *g_ep0;
static uint64_t g_ep0Phys;

// Every stage starts at TRB slot 0: prepare_one_trb advances trb_enqueue for
// chained transfers only, and start_trans always names ep0_trb_addr. An IN data
// stage that spans several packets chains one TRB per packet from slot 0, which
// stays clear of the data buffer below.
#define EP0_TRB       0x000u
#define EP0_STAGE_BUF 0x100u

// 0 setup armed, 1 data armed, 2 waiting for XferNotReady(status), 3 status armed
static int g_ep0_stage;
static bool g_ep0_three;   // three-stage control transfer (wLength != 0)
static bool g_ep0_in;      // data stage runs device -> host
static unsigned g_ep0_status_dep;  // dep the status stage belongs to
static unsigned g_ep0_armed;       // bytes handed to the data TRBs
static unsigned g_ep0Trbs;         // TRBs in the armed chain, 0 when none
static bool g_ep0_zlp;             // IN data ended on a maxpacket boundary
static int g_configured;
static unsigned g_last_link;
static unsigned g_usbEvents;   // ring entries consumed since the pull-up went up
static unsigned g_ep0Setups;   // SETUP packets parsed by ep0Setup()

// This file only appends to the console ring: the periodic writer thread in
// NativeUfs owns UFS and publishes the ring every 2 s. Keeping the flush out of the
// control-transfer path means a hang here can no longer be confused with a dead log,
// and no UFS write can sit inside a handshake the host is timing.
// Each checkpoint also paints its id at the bottom of the panel, below the kprintf
// scroll area and above the panic rows.
#define USB_STEP_Y 1860
#define USB_REQ_Y  1920
#define USB_LEN_Y  1980
static void usbStep(unsigned id)
{
  nabu_mark_num("usb:", id, USB_STEP_Y);
}

// CDC-ACM: interface 0 is the control interface (class 2/2/1) with a 16-byte
// interrupt notification endpoint; interface 1 is the data interface (class 10)
// with a bulk pair. Windows binds this to usbser.sys without an .inf.
static const uint8_t kDeviceDesc[18] = {
  18, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 64,
  0x6b, 0x1d, 0x04, 0x01, 0x00, 0x01,
  1, 2, 3, 1,                       // iManufacturer, iProduct, iSerial, bNumConfigurations
};

static const uint8_t kConfigDesc[67] = {
  9, 0x02, 67, 0, 2, 1, 0, 0x80, 0x32,
  9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,   // CDC control, subclass ACM, protocol AT
  5, 0x24, 0x00, 0x10, 0x01,               // header, bcdCDC 1.10
  5, 0x24, 0x01, 0x00, 0x01,               // call management -> data interface 1
  4, 0x24, 0x02, 0x06,                     // ACM: line coding + send break
  5, 0x24, 0x06, 0x00, 0x01,               // union: control 0, data 1
  7, 0x05, 0x82, 0x03, 16, 0, 16,          // EP2 IN, interrupt, 16 bytes
  9, 0x04, 1, 0, 2, 0x0a, 0x00, 0x00, 0,   // CDC data
  7, 0x05, 0x81, 0x02, 64, 0, 0,           // EP1 IN, bulk, 64
  7, 0x05, 0x02, 0x02, 64, 0, 0,           // EP1 OUT, bulk, 64
};

static const char *const kUsbStrings[4] = { "", "nabu", "nabu console", "0000000001" };

static uint8_t *ep0Buf(void)
{
  return reinterpret_cast<uint8_t *>(const_cast<uint32_t *>(g_ep0 + EP0_STAGE_BUF / 4));
}

// dwc3_send_gadget_ep_cmd: at HS/FS SUSPHY and ENBLSLPM must be clear for the
// duration of an endpoint command, then restored. The poll is bounded because a
// command that never completes must not stop the boot.
static int epCmd(unsigned dep, uint32_t cmd, uint32_t p0, uint32_t p1, uint32_t p2)
{
  const uint32_t cfg = rd(DWC3_GUSB2PHYCFG(0));
  const uint32_t drop = cfg & (DWC3_GUSB2PHYCFG_SUSPHY | DWC3_GUSB2PHYCFG_ENBLSLPM);
  if (drop) { wr(DWC3_GUSB2PHYCFG(0), cfg & ~drop); }
  wr(DWC3_DEPCMDPAR0(dep), p0);
  wr(DWC3_DEPCMDPAR1(dep), p1);
  wr(DWC3_DEPCMDPAR2(dep), p2);
  wr(DWC3_DEPCMD(dep), cmd | DWC3_DEPCMD_CMDACT);
  int status = -1;
  for (unsigned i = 0; i < 20000; ++i) {
    const uint32_t v = rd(DWC3_DEPCMD(dep));
    if (!(v & DWC3_DEPCMD_CMDACT)) {
      status = static_cast<int>(DWC3_DEPCMD_STATUS(v));
      break;
    }
    __asm__ volatile("yield" ::: "memory");
  }
  if (drop) { wr(DWC3_GUSB2PHYCFG(0), cfg); }
  return status;
}

// Every stage is one ring at the base address; LST on the last TRB stops the core
// from fetching past it. HWO is written last, behind the barrier in ep0Start, so
// the core never sees a half-written request.
static int ep0Start(unsigned dep)
{
  __asm__ volatile("dsb sy" ::: "memory");
  return epCmd(dep, DWC3_DEPCMD_STARTTRANSFER,
               static_cast<uint32_t>(g_ep0Phys >> 32),
               static_cast<uint32_t>(g_ep0Phys), 0);
}

// HWO is deliberately left clear here. The core is free to fetch the head TRB the
// instant HWO goes up and, with CHN set, will walk straight into the next entry --
// which for a multi-packet reply is still the previous transfer's TRB (HWO clear).
// A core retrying that fetch never releases the bus, so the CPU stops dead inside
// this function: that is the shape of every run that ended on the config descriptor.
static void ep0TrbAt(unsigned slot, uint64_t buf, uint32_t size, uint32_t type, bool last)
{
  volatile uint32_t *t = g_ep0 + slot * 4u;
  t[0] = static_cast<uint32_t>(buf);
  t[1] = static_cast<uint32_t>(buf >> 32);
  t[2] = size;
  uint32_t ctrl = type | DWC3_TRB_CTRL_ISP_IMI;
  ctrl |= last ? (DWC3_TRB_CTRL_IOC | DWC3_TRB_CTRL_LST) : DWC3_TRB_CTRL_CHN;
  t[3] = ctrl;
}

// Hands the finished chain to the core: every entry is on DRAM before the head
// becomes valid, so a chain walk can never see a half-written or stale entry.
static void ep0TrbHead(void)
{
  __asm__ volatile("dsb sy" ::: "memory");
  volatile uint32_t *t = g_ep0;
  t[3] = t[3] | DWC3_TRB_CTRL_HWO;
  __asm__ volatile("dsb sy" ::: "memory");
}

static void ep0Trb(uint64_t buf, uint32_t size, uint32_t type)
{
  g_ep0Trbs = 1;
  ep0TrbAt(0, buf, size, type, true);
  ep0TrbHead();
}

static void ep0ArmSetup(void)
{
  g_ep0_stage = 0;
  g_ep0_armed = 0;
  ep0Trb(g_ep0Phys + EP0_TRB, 8, DWC3_TRBCTL_CONTROL_SETUP);
  (void)ep0Start(0);
}

static void ep0Data(unsigned dep, unsigned bufOff, uint32_t len)
{
  g_ep0_stage = 1;
  g_ep0_armed = len;
  if (dep != 0 && len > 64) {
    // The vendor hands the core one TRB for the whole IN length, which wedged
    // this core on the first multi-packet reply (the 67-byte config descriptor).
    // Chain one TRB per maxpacket instead: the core then only has to send each
    // packet as it is described and never splits a TRB itself.
    const unsigned packets = (len + 63) / 64;
    g_ep0Trbs = packets;
    for (unsigned i = 0; i < packets; ++i) {
      const uint32_t sent = i * 64;
      const uint32_t chunk = len - sent > 64 ? 64 : len - sent;
      ep0TrbAt(i, g_ep0Phys + bufOff + sent, chunk, DWC3_TRBCTL_CONTROL_DATA,
               i + 1 == packets);
    }
  } else {
    ep0TrbAt(0, g_ep0Phys + bufOff, len, DWC3_TRBCTL_CONTROL_DATA, true);
    g_ep0Trbs = 1;
  }
  // Bracket the handoff: these two lines land before HWO, so a run that stops here
  // separates "died programming the chain" from "died with the core owning it".
  nabu_log_hex("usb:trbc=", g_ep0[3]);
  if (g_ep0Trbs > 1) { nabu_log_hex("usb:trbc2=", g_ep0[7]); }
  ep0TrbHead();
  // A wedge inside STARTTRANSFER leaves no event behind, so the arm and its
  // command status must already be on the medium before the core touches DRAM.
  usbStep(3);
  nabu_log_hex("usb:data-arm=", (static_cast<unsigned>(dep) << 24) |
                                (g_ep0Trbs << 20) | len);
  usbStep(4);
  const int st = ep0Start(dep);
  usbStep(st == 0 ? 5 : 15);
  nabu_log_hex("usb:data-cmd=", static_cast<unsigned>(st));
}

static void ep0Status(unsigned dep)
{
  g_ep0_stage = 3;
  ep0Trb(g_ep0Phys + EP0_TRB, 0,
         g_ep0_three ? DWC3_TRBCTL_CONTROL_STATUS3 : DWC3_TRBCTL_CONTROL_STATUS2);
  usbStep(7);
  nabu_log_hex("usb:st-arm=", static_cast<unsigned>(dep));
  const int st = ep0Start(dep);
  usbStep(st == 0 ? 8 : 16);
  nabu_log_hex("usb:st-cmd=", static_cast<unsigned>(st));
}

// dwc3_ep0_stall_and_restart: SetStall aborts whatever control transfer is in
// the core, and the endpoint must be listening for SETUP again before returning.
static void ep0Restart(void)
{
  (void)epCmd(0, DWC3_DEPCMD_SETSTALL, 0, 0, 0);
  (void)epCmd(0, DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
  ep0ArmSetup();
}

static unsigned ep0StringDesc(unsigned idx, uint8_t *out)
{
  if (idx == 0) {
    out[0] = 4; out[1] = 3; out[2] = 0x09; out[3] = 0x04;   // en-US
    return 4;
  }
  if (idx >= 4) { return 0; }
  const char *s = kUsbStrings[idx];
  unsigned n = 0;
  while (s[n] != '\0' && n < 63) { ++n; }
  out[0] = static_cast<uint8_t>(2 + 2 * n);
  out[1] = 3;
  for (unsigned i = 0; i < n; ++i) {
    out[2 + 2 * i] = static_cast<uint8_t>(s[i]);
    out[3 + 2 * i] = 0;
  }
  return 2 + 2 * n;
}

// dep2 = EP1 OUT, dep3 = EP1 IN, dep5 = EP2 IN. FIFO 0 is the shared pool the
// core runs with in HS mode, so no TX FIFO resplit is needed for these.
static void ep0EnableDataEps(void)
{
  const uint32_t common = DWC3_DEPCFG_XFER_COMPLETE_EN | DWC3_DEPCFG_XFER_IN_PROGRESS_EN;
  (void)epCmd(2, DWC3_DEPCMD_SETEPCONFIG,
              DWC3_DEPCFG_EP_TYPE(2) | DWC3_DEPCFG_MAX_PACKET_SIZE(64),
              common | DWC3_DEPCFG_EP_NUMBER(2), 0);
  (void)epCmd(3, DWC3_DEPCMD_SETEPCONFIG,
              DWC3_DEPCFG_EP_TYPE(2) | DWC3_DEPCFG_MAX_PACKET_SIZE(64),
              common | DWC3_DEPCFG_EP_NUMBER(3), 0);
  (void)epCmd(5, DWC3_DEPCMD_SETEPCONFIG,
              DWC3_DEPCFG_EP_TYPE(3) | DWC3_DEPCFG_MAX_PACKET_SIZE(16),
              common | DWC3_DEPCFG_EP_NUMBER(5), 0);
  wr(DWC3_DALEPENA, rd(DWC3_DALEPENA) | DWC3_DALEPENA_EP(2) |
                    DWC3_DALEPENA_EP(3) | DWC3_DALEPENA_EP(5));
  usbStep(13);
  nabu_log_hex("ep0:dalep2=", rd(DWC3_DALEPENA));
}

static void ep0Setup(void)
{
  const uint32_t w0 = g_ep0[0];
  const uint32_t w1 = g_ep0[1];
  const uint8_t rt = w0 & 0xff;
  const uint8_t req = (w0 >> 8) & 0xff;
  const uint16_t val = (w0 >> 16) & 0xffff;
  const uint16_t idx = w1 & 0xffff;
  const uint16_t len = (w1 >> 16) & 0xffff;

  ++g_ep0Setups;
  gDwc3Setups = gDwc3Setups + 1;
  usbStep(2);
  nabu_mark_num("req:", static_cast<uint32_t>(rt) | (static_cast<uint32_t>(req) << 8), USB_REQ_Y);
  nabu_mark_num("len:", len, USB_LEN_Y);
  nabu_log_hex("usb:setup=", static_cast<uint32_t>(rt) | (static_cast<uint32_t>(req) << 8));
  nabu_log_hex("usb:setupw=", (static_cast<uint32_t>(val) << 16) | idx);
  nabu_log_hex("usb:setupl=", len);
  /* No UFS access on this path. Run 75 ended on this exact line with no
   * data-arm after it: the snapshot was the only thing this path did before
   * arming, so it wedged inside the handshake the host is timing and left
   * nabu_log_busy set, muting every later flush. The watchdog now asks the
   * log thread for a snapshot and resets the board either way. */
  g_ep0_three = len != 0;
  g_ep0_in = g_ep0_three && (rt & 0x80) != 0;
  g_ep0_zlp = false;
  // dwc3: direction = !ep0_expect_in, so an IN data stage takes its status on
  // dep0 and everything else takes it on dep1.
  g_ep0_status_dep = g_ep0_in ? 0u : 1u;

  uint8_t *out = ep0Buf();
  unsigned resp = 0;
  bool stall = false;

  if ((rt & 0x60) == 0) {                      // standard
    switch (req) {
    case 6: {                                  // GET_DESCRIPTOR
      const unsigned type = val >> 8;
      if (type == 1) {
        memcpy(out, kDeviceDesc, sizeof(kDeviceDesc));
        resp = sizeof(kDeviceDesc);
      } else if (type == 2) {
        memcpy(out, kConfigDesc, sizeof(kConfigDesc));
        resp = sizeof(kConfigDesc);
      } else if (type == 3) {
        resp = ep0StringDesc(val & 0xff, out);
        if (resp == 0) { stall = true; }
      } else {
        stall = true;                        // no device qualifier at HS-only
      }
      break;
    }
    case 5:                                    // SET_ADDRESS
      wr(DWC3_DCFG, (rd(DWC3_DCFG) & ~DWC3_DCFG_DEVADDR_MASK) |
                    DWC3_DCFG_DEVADDR(val & 0x7f));
      // Read back immediately: if the core drops the write, the device stays on
      // address 0 and every packet the host sends after this point is out of range.
      nabu_log_hex("usb:dcfg-set=", rd(DWC3_DCFG));
      break;
    case 9:                                    // SET_CONFIGURATION
      g_configured = val;
      if (val != 0) { ep0EnableDataEps(); }
      nabu_log_hex("usb:configured=", static_cast<unsigned>(val));
      break;
    case 8:                                    // GET_CONFIGURATION
      out[0] = static_cast<uint8_t>(g_configured);
      resp = 1;
      break;
    case 0:                                    // GET_STATUS
      out[0] = 0; out[1] = 0;
      resp = 2;
      break;
    case 1:                                    // CLEAR_FEATURE
    case 3:                                    // SET_FEATURE
    case 11:                                   // SET_INTERFACE
      break;
    default:
      stall = true;
      break;
    }
  } else if (req == 0x21) {                    // CDC GET_LINE_CODING: 115200 8N1
    out[0] = 0x00; out[1] = 0xC2; out[2] = 0x01; out[3] = 0x00;
    out[4] = 0x00; out[5] = 0x00; out[6] = 0x08;
    resp = 7;
  } else if (req == 0x20 || req == 0x22 || req == 0x23) {
    // SET_LINE_CODING (data arrives in the OUT stage), SET_CONTROL_LINE_STATE,
    // SEND_BREAK: accepted, nothing to do.
  } else {
    stall = true;
  }

  if (stall) {
    usbStep(12);
    nabu_log_puts("usb:stall\n");
    ep0Restart();
    return;
  }
  // Last line before the data stage is armed, with the direction and length this
  // side chose: the host asked for 255 bytes of config descriptor and the reply is
  // 67, so this is the only record of what was about to be handed to the core.
  nabu_log_hex("usb:resp=", (static_cast<unsigned>(g_ep0_in) << 17) |
                            (static_cast<unsigned>(g_ep0_three) << 16) | resp);
  if (resp > len) { resp = len; }

  if (!g_ep0_three) {
    // Two-stage: the status phase is on demand, wait for XferNotReady(status).
    g_ep0_stage = 2;
    return;
  }
  if (g_ep0_in) {
    // A response that ends exactly on the maxpacket boundary needs a trailing
    // zero-length packet, or the host keeps polling for the rest of the transfer
    // (the vendor handles this through urb->zero).
    g_ep0_zlp = resp != 0 && (resp % 64) == 0;
    ep0Data(1, EP0_STAGE_BUF, resp);
  } else {
    // ISP_IMI is set like the vendor's prepare_one_trb, so a short packet does
    // not end the TRB; arm exactly wLength and the length itself completes it.
    ep0Data(0, EP0_STAGE_BUF, len > 64 ? 64 : len);
  }
}

static void usbHandleDep(uint32_t ev)
{
  const unsigned dep = (ev >> 1) & 0x1f;
  const unsigned kind = (ev >> 6) & 0xf;
  const unsigned status = (ev >> 12) & 0xf;

  if (kind == DWC3_DEPEVT_XFERNOTREADY) {
    if (status == DEPEVT_STATUS_CONTROL_STATUS && g_ep0_stage == 2) {
      // The vendor arms the status stage on the dep the event names, so trust
      // that first and fall back to the computed direction only if it is odd.
      nabu_log_hex("usb:nrdy-status=", dep);
      ep0Status(dep <= 1 ? dep : g_ep0_status_dep);
    } else {
      nabu_log_hex("usb:nrdy=", (dep << 8) | (kind << 4) | status);
    }
    return;
  }
  if (kind != DWC3_DEPEVT_XFERCOMPLETE) {
    nabu_log_hex("usb:epevt=", (dep << 8) | (kind << 4) | status);
    return;
  }

  if (g_ep0_stage == 0) {
    ep0Setup();
  } else if (g_ep0_stage == 1) {
    // A chained IN stage reports TRBSTS on the TRB the core stopped at, so poll
    // every entry instead of only slot 0 the way the one-TRB ring could.
    uint32_t last = 0;
    bool pending = false;
    for (unsigned i = 0; i < g_ep0Trbs; ++i) {
      last = g_ep0[i * 4u + 2];
      if (DWC3_TRB_SIZE_TRBSTS(last) == DWC3_TRBSTS_SETUP_PENDING) { pending = true; }
    }
    usbStep(pending ? 11 : 6);
    nabu_log_hex("usb:data-done=", (g_ep0_armed - (last & DWC3_TRB_SIZE_MASK)) |
                                  (g_ep0_in ? 0x10000u : 0u) | (g_ep0Trbs << 20));
    nabu_log_hex("usb:data-size=", last);
    if (pending) {
      nabu_log_puts("usb:setup-pending\n");
      ep0ArmSetup();                           // host moved on to a new SETUP
      return;
    }
    if (g_ep0_zlp) {
      g_ep0_zlp = false;
      nabu_log_puts("usb:zlp\n");
      usbStep(10);
      ep0Data(1, EP0_STAGE_BUF, 0);
      return;
    }
    g_ep0_stage = 2;                            // status is on demand
  } else if (g_ep0_stage == 3) {
    usbStep(9);
    nabu_log_puts("usb:xfer-done\n");
    ep0ArmSetup();
  } else {
    nabu_log_hex("usb:evt-odd=", ev);
  }
}

static void usbPollEvents(void)
{
  gDwc3Ticks = gDwc3Ticks + 1;                  // watchdog heartbeat, see NativeUfs
  for (unsigned guard = 0; guard < 64; ++guard) {
    const uint32_t count = rd(DWC3_GEVNTCOUNT(0)) & 0xfffcu;
    if (count == 0) {
      return;
    }
    nabu_log_hex("usb:evtcnt2=", count);
    if (count > 4096) {
      // The core wrapped past the whole ring, so nothing readable is left to
      // resync against; drop it the way dwc3's HUGE_EVCNT path does.
      g_evtOff = (g_evtOff + count) % 4096;
      wr(DWC3_GEVNTCOUNT(0), count);
      continue;
    }
    // The ring has no read-pointer register: the software offset is the only
    // read cursor and the core only advances when we ack. Always reading from
    // 0 re-reads the oldest event and silently drops every newer one.
    unsigned left = count;
    while (left >= 4) {
      const uint32_t ev = g_evt[g_evtOff / 4];
      ++g_usbEvents;
      gDwc3Events = gDwc3Events + 1;
      nabu_log_hex("usb:ev0=", ev);
      // No flush here. This used to write and sync 64 KiB of UFS before every
      // single event, inside a handshake the host gives up on after ~5 s -- run 65
      // wedged exactly there. The period counter above tells the writer thread in
      // NativeUfs that something happened; it publishes the ring within 2 s, off
      // this path, so a slow flush can delay the log but never the transfer.
      if (ev & 1u) {                            // device event: type[11:8], info[24:16]
        const unsigned type = (ev >> 8) & 0xf;
        const unsigned info = (ev >> 16) & 0x1ff;
        nabu_log_hex("usb:devt=", (type << 16) | info);
        if (type == DWC3_DEV_EVENT_RESET) {
          nabu_log_puts("usb:devrst\n");
          wr(DWC3_DCFG, rd(DWC3_DCFG) & ~DWC3_DCFG_DEVADDR_MASK);
          g_configured = 0;
          ep0Restart();
        } else if (type == DWC3_DEV_EVENT_LINK_STATUS) {
          g_last_link = info;
        }
      } else {
        usbHandleDep(ev);
      }
      g_evtOff = (g_evtOff + 4) % 4096;
      left -= 4;
    }
    wr(DWC3_GEVNTCOUNT(0), count);
  }
}

static void ep0Program(void)
{
  (void)epCmd(0, DWC3_DEPCMD_DEPSTARTCFG, 0, 0, 0);
  for (unsigned dep = 0; dep < 32; ++dep) {
    (void)epCmd(dep, DWC3_DEPCMD_SETTRANSFRESOURCE,
                DWC3_DEPXFERCFG_NUM_XFER_RES(1), 0, 0);
  }
  const uint32_t cfg0 = DWC3_DEPCFG_EP_TYPE(0) | DWC3_DEPCFG_MAX_PACKET_SIZE(64);
  const int st = epCmd(0, DWC3_DEPCMD_SETEPCONFIG, cfg0,
                       DWC3_DEPCFG_XFER_COMPLETE_EN | DWC3_DEPCFG_XFER_NOT_READY_EN |
                         DWC3_DEPCFG_EP_NUMBER(0), 0);
  // EP0's two directions are separate physical endpoints: the vendor's
  // __dwc3_gadget_start enables eps[0] and eps[1] the same way, so dep1 needs
  // its own config and its own DALEPENA bit or every IN stage (the whole device
  // descriptor) never leaves the core.
  const int st1 = epCmd(1, DWC3_DEPCMD_SETEPCONFIG, cfg0,
                        DWC3_DEPCFG_XFER_COMPLETE_EN | DWC3_DEPCFG_XFER_NOT_READY_EN |
                          DWC3_DEPCFG_EP_NUMBER(1), 0);
  wr(DWC3_DALEPENA, rd(DWC3_DALEPENA) | DWC3_DALEPENA_EP(0) | DWC3_DALEPENA_EP(1));
  nabu_log_hex("ep0:cfg=", static_cast<unsigned>(st));
  nabu_log_hex("ep0:cfg-in=", static_cast<unsigned>(st1));
  nabu_log_hex("ep0:dalep=", rd(DWC3_DALEPENA));
  ep0ArmSetup();
  usbStep(1);
  nabu_log_hex("ep0:arm=", rd(DWC3_DEPCMD(0)));
}

static bool ep0Init(void)
{
  g_ep0Dma = IOBufferMemoryDescriptor::inTaskWithOptions(
    kernel_task,
    static_cast<IOOptionBits>(kIODirectionInOut) |
      static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
      static_cast<IOOptionBits>(kIOMemoryMapperNone),
    4096, 4096);
  if (g_ep0Dma == nullptr) {
    nabu_log_puts("ep0:dma-fail\n");
    return false;
  }
  IOByteCount contiguous = 0;
  g_ep0Phys = g_ep0Dma->getPhysicalSegment(0, &contiguous, kIOMemoryMapperNone);
  g_ep0 = reinterpret_cast<volatile uint32_t *>(
    mapPhys(g_ep0Phys, 4096, g_ep0Keep));
  nabu_log_hex("ep0:pa=", g_ep0Phys);
  nabu_log_hex("ep0:len=", contiguous);
  if (g_ep0 == nullptr || g_ep0Phys == 0 || (g_ep0Phys & 0xfff) || contiguous < 4096) {
    nabu_log_puts("ep0:dma-bad\n");
    return false;
  }
  for (unsigned i = 0; i < 1024; ++i) { g_ep0[i] = 0; }
  ep0Program();
  return true;
}

static int dwc3Bringup(void)
{
  if (g_bringup_done) {
    return 0;
  }
  g_bringup_done = 1;
  nabu_log_puts("dwc3:start\n");
  nabu_log_puts("usb:marker\n");
  logProbe("p:enter\n");

  int enable = 1, init = 1, keep = 0;
  (void)PE_parse_boot_argn("nabu-usb", &enable, sizeof(enable));
  (void)PE_parse_boot_argn("nabu-usb-init", &init, sizeof(init));
  (void)PE_parse_boot_argn("nabu-usb-keep", &keep, sizeof(keep));
  if (!enable) {
    nabu_log_puts("dwc3:off\n");
    return 0;
  }

  if (keep) {
    // Fastboot drives this same controller, so the bootloader has already
    // powered the PHY and wired up QSCRATCH. Read what it left behind instead of
    // resetting the PHY back into a state XNU cannot bring up.
    OSSharedPtr<IOMemoryMap> kKeep;
    volatile uint32_t *k =
      reinterpret_cast<volatile uint32_t *>(mapPhys(NABU_USB0_CORE_BASE, NABU_USB0_CORE_SIZE, kKeep));
    if (k == nullptr) {
      nabu_log_puts("keep:map-fail\n");
      return -1;
    }
    nabu_log_puts("keep:probe\n");
    nabu_log_hex("keep:gctl=", k[DWC3_GCTL / 4]);
    nabu_log_hex("keep:dctl=", k[DWC3_DCTL / 4]);
    nabu_log_hex("keep:dsts=", k[DWC3_DSTS / 4]);
    nabu_log_hex("keep:dcfg=", k[DWC3_DCFG / 4]);
    nabu_log_hex("keep:g2cfg=", k[DWC3_GUSB2PHYCFG(0) / 4]);
    nabu_log_hex("keep:g3ctl=", k[DWC3_GUSB3PIPECTL(0) / 4]);
    nabu_log_hex("keep:osts=", k[DWC3_OSTS / 4]);
    nabu_log_hex("keep:qsg=", k[QSCRATCH_GENERAL_CFG / 4]);
    nabu_log_hex("keep:qsh=", k[QSCRATCH_HS_PHY_CTRL / 4]);
    // What did the bootloader leave in the PHY? fastboot's USB ran on this PHY,
    // so if it kept a PLL/analog configuration we want that on the record.
    {
      OSSharedPtr<IOMemoryMap> pKeep;
      volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(
        mapPhys(QUSB2_BASE, 0x1000, pKeep));
      if (p == nullptr) {
        nabu_log_puts("keep:phy-map-fail\n");
      } else {
        for (unsigned off = 0; off <= 0x38; off += 4) {
          nabu_log_hex("keep:pll=", (off << 16) | (p[off / 4] & 0xffu));
        }
        for (unsigned off = 0x3c; off <= 0xa0; off += 4) {
          nabu_log_hex("keep:reg=", (off << 16) | (p[off / 4] & 0xffu));
        }
        nabu_log_hex("keep:reg=", (0xb4u << 16) | (p[0xb4 / 4] & 0xffu));
      }
    }
    // TLMM: the PHY's vbusvldext pin is one of these. Dump config + level for
    // the low GPIOs so we can see what the bootloader left driven high.
    {
      OSSharedPtr<IOMemoryMap> tKeep;
      volatile uint32_t *t = reinterpret_cast<volatile uint32_t *>(
        mapPhys(0x03D00000ULL, 0x60000, tKeep));
      if (t == nullptr) {
        nabu_log_puts("keep:tlmm-fail\n");
      } else {
        for (unsigned g = 0; g < 96; ++g) {
          const uint32_t cfg = t[(g * 0x1000) / 4];
          const uint32_t io = t[(g * 0x1000 + 4) / 4];
          if (cfg == 0 && io == 0) { continue; }
          nabu_log_hex("keep:gcfg=", (g << 16) | (cfg & 0xffffu));
          nabu_log_hex("keep:gio=", (g << 16) | (io & 0x3u));
        }
      }
    }    return 1;
  }

  rpmhProbe();
  {
    // PMIC arbiter: probe the block before building any channel/mapping logic.
    OSSharedPtr<IOMemoryMap> spmiKeep;
    volatile uint32_t *spmi =
      reinterpret_cast<volatile uint32_t *>(mapPhys(0x0C440000ULL, 0x2000, spmiKeep));
    if (spmi == nullptr) {
      nabu_log_puts("spmi:map-fail\n");
    } else {
      nabu_log_hex("spmi:ver=", spmi[0]);
      // PMIC arbiter v5 (see drivers/spmi/spmi-pmic-arb.c): the APID table is
      // one entry per APID at 0x900 + 4*apid, and each entry holds the PPID in
      // bits[23:8]. This is NOT the v2 16-level tree at 0xB00 that the earlier
      // code walked, which is why every lookup reported valid=0.
      for (unsigned a = 0; a < 16; ++a) {
        nabu_log_hex("spmi:mt=", spmi[(0x900 / 4) + a]);
      }
      // The APID table can be up to 512 entries; the charger's APID may live past
      // 256. Read a bounded slice and stop at the first unimplemented entry.
      for (unsigned a = 256; a < 512; ++a) {
        uint32_t ent = spmi[(0x900 / 4) + a];
        if (ent == 0xffffffffu) { break; }
        if (!ent) { continue; }
        nabu_log_hex("spmi:tab2=", (a << 16) | ((ent >> 8) & 0xfffu));
      }
      static const unsigned kSpmiRegs[] = {
        0x1310, 0x1308, 0x1544,
        0xC840, 0xC846, 0xC940, 0xC946,
      };
      for (unsigned r = 0; r < sizeof(kSpmiRegs) / sizeof(kSpmiRegs[0]); ++r) {
        const unsigned addr = kSpmiRegs[r];
        const unsigned ppid = (2u << 8) | ((addr >> 8) & 0xffu);  // PM8150B = sid 2
        // Dump every populated APID table entry once (0x900..0xCFF is the whole
        // 0x400-byte table; going beyond it wedges the arbiter, run 20).
        if (r == 0) {
          for (unsigned a = 0; a < 256; ++a) {
            uint32_t ent = spmi[(0x900 / 4) + a];
            if (!ent) { continue; }
            nabu_log_hex("spmi:tab=", (a << 16) | ((ent >> 8) & 0xfffu));
          }
        }
        unsigned apid = 0xffffffffU;
        // The charger's APID sits at 266 (PPID 0x210) and the PD PHY's at 273
        // (0x217); the old 128-entry scan never reached them.
        for (unsigned a = 0; a < 512; ++a) {
          uint32_t ent = spmi[(0x900 / 4) + a];
          if (!ent) { continue; }
          if (((ent >> 8) & 0xfffu) == ppid) { apid = a; break; }
        }
        nabu_log_hex("spmi:ppid=", ppid);
        nabu_log_hex("spmi:apid=", apid == 0xffffffffU ? 0xfffu : apid);
        if (apid == 0xffffffffU) {
          continue;
        }
        // v5 observer channel: obsrvr base + 0x80*apid (ee = 0). The arbiter's
        // DT gives obsrvr = 0x0E600000; adding the offset to `core` wedged it.
        OSSharedPtr<IOMemoryMap> chanKeep;
        const uint64_t chanPhys = 0x0E600000ULL + 0x80ULL * apid;
        const uint64_t pagePhys = chanPhys & ~0xFFFULL;
        volatile uint32_t *page = reinterpret_cast<volatile uint32_t *>(
          mapPhys(pagePhys, 0x1000, chanKeep));
        if (page == nullptr) {
          nabu_log_puts("spmi:chan-fail\n");
          continue;
        }
        volatile uint32_t *chan = page + (chanPhys - pagePhys) / 4;
        // fmt_cmd: opcode<<27 | sid<<20 | addr<<4 | byte_count. The earlier code
        // dropped the SID field, so every read went to sid 0.
        chan[0] = (15u << 27) | ((2u & 0xfu) << 20) | ((addr & 0xffu) << 4);
        unsigned done = 0;
        for (unsigned i = 0; i < 20000; ++i) {
          if (chan[2] & 1u) { done = 1; break; }
          __asm__ volatile("yield" ::: "memory");
        }
        nabu_log_hex("spmi:done=", done);
        if (done) {
          nabu_log_hex("spmi:val=", chan[6] & 0xffu);
        }
      }
      // Probe: drive PM8150B GPIO1's software output high (DIG_OUT_SRC_CTL bit0)
      // and see whether the PHY finally reports VBUS. APID 295 came from the table.
      nabu_log_hex("spmi:gpio1w=", spmiWrite(0x127u, 0xC044u, 1u) ? 1u : 0u);
      // The DT's extcon_usb1 node names vbus-out-gpio = <&pm8150b_gpios 9>.
      // PMIC GPIO MODE_CTL is {dir[6:4], func[3:1], value[0]}, and GPIO9 reads 1,
      // i.e. direction 0 = INPUT. Set direction 1 (output) with value 1.
      nabu_log_hex("spmi:vbusout=", spmiWrite(0x12cu, 0xC840u, 0x11u) ? 1u : 0u);
      // Android runs this board with TYPE_C_MODE_CFG = 0x10 (TWRP dmesg); XNU
      // finds 0x02 there. Put it back to the Android value, since that value is
      // the one the working stack uses.
      nabu_log_hex("spmi:tcmode=", spmiWrite(0x10fu, 0x1544u, 0x10u) ? 1u : 0u);
    }
  }
  // Bootloader already had these rails up for fastboot's own USB session, so
  // leave them exactly as it left them. Re-voting from here is the last thing
  // that could have put the PHY supply in a strange state.
  int voteRails = 0;
  (void)PE_parse_boot_argn("nabu-usb-rails", &voteRails, sizeof(voteRails));
  nabu_log_hex("rpmh:vote-rails=", voteRails);
  if (voteRails) {
    (void)rpmhUsbPhyRails();
  }
  // RPMh completion means the RSC accepted the command, not that the PMIC
  // finished ramping. Give the rails time before the PHY AHB is touched.
  for (unsigned i = 0; i < 500000; ++i) { __asm__ volatile("yield" ::: "memory"); }

  const unsigned failed = gccUsbPowerOn();
  nabu_log_hex("dwc3:clkfail=", failed);
  if (failed == 0xff) {
    nabu_log_puts("dwc3:gcc-map-bad\n");
    return -1;
  }

  // Reset pulse, then the femto init, then power-on (clear SLEEPM).
  if (g_gcc != nullptr) {
    gwr(GCC_QUSB2PHY_PRIM_BCR, 0x1);
    for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
    gwr(GCC_QUSB2PHY_PRIM_BCR, 0x0);
  }
  g_phy = reinterpret_cast<volatile uint32_t *>(mapPhys(QUSB2_BASE, 0x1000, g_phyKeep));
  // GPIO 38 is the only pin whose TLMM function is "usb_phy"
  // (pinctrl-sm8150.c: PINGROUP(38, SOUTH, usb_phy, ...), so funcs = {gpio,
  // SOUTH, usb_phy} and usb_phy is func 2). The bootloader hands it over on
  // func 1, which is why the PHY's VBUS side never sees the cable. Switch it
  // back to usb_phy before touching the PHY.
  // TLMM read-only: pins 113 and 123 are the `usb2phy_ac` function, the only
  // other USB2-PHY-related pinmux on this SoC. Reading TLMM is safe; only the
  // writes wedged the board (and are not done here).
  {
    static const unsigned kPins[] = { 113u, 123u };
    for (unsigned i = 0; i < 2; ++i) {
      const unsigned g = kPins[i];
      OSSharedPtr<IOMemoryMap> pinKeep;
      volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(
        mapPhys(0x03000000ULL + 0x1000ULL * g, 0x1000, pinKeep));
      if (p == nullptr) { continue; }
      nabu_log_hex("tlmm:cfg=", (g << 16) | (p[0] & 0xffffu));
      nabu_log_hex("tlmm:io=", (g << 16) | (p[1] & 0x3u));
    }
  }
  // QUP access removed: reading i2c@890000 without first enabling its GCC
  // clocks (0x5200c bit6/7/11) stalls the AXI bus and wedges the board.
  if (g_phyInitMode) { (void)femtoPhyInit(); }
  // No settle delay of any kind. Both CNTPCT and IODelay are unusable this early
  // (the counter does not advance yet, so the loop never ends) and each attempt
  // wedged the boot at IOFindBSRoot. Bounded yield loops only, never a wait on
  // a clock that may not be running.
  for (unsigned i = 0; i < 200000; ++i) { __asm__ volatile("yield" ::: "memory"); }


  const uintptr_t va = mapPhys(NABU_USB0_CORE_BASE, NABU_USB0_CORE_SIZE, g_keep);
  g_regs = reinterpret_cast<volatile uint32_t *>(va);
  if (g_regs == nullptr) {
    nabu_log_puts("dwc3:map-failed\n");
    return -1;
  }
  nabu_log_hex("dwc3:cap=", rd(0x0));
  nabu_log_hex("dwc3:gsbus=", rd(DWC3_GSBUSCFG0));
  nabu_log_hex("dwc3:gctl=", rd(DWC3_GCTL));
  nabu_log_hex("dwc3:guid=", rd(DWC3_GUID));
  nabu_log_hex("dwc3:hw0=", rd(DWC3_GHWPARAMS0));
  const uint32_t sid = rd(DWC3_GSNPSID);
  nabu_log_hex("dwc3:sid=", sid);
  const uint32_t family = sid & DWC3_GSNPSID_MASK;
  if (family != DWC3_GSNPSID_DWC_USB3 && family != DWC3_GSNPSID_DWC_USB31 &&
      family != DWC3_GSNPSID_DWC_USB31_3331) {
    nabu_log_puts("dwc3:id-bad\n");
    return -1;
  }
  nabu_log_hex("dwc3:params0=", rd(DWC3_GHWPARAMS0));

  if (!init) {
    nabu_log_puts("dwc3:stage1-done\n");
    return 1;
  }

  // What the bootloader left in the UTMI clock mux decides whether the vendor's
  // PIPE_UTMI_CLK_DIS toggle is needed at all -- if CLK_SEL/PHYSTATUS_SW are
  // already set, the clock is already routed and touching CLK_DIS is pointless risk.
  nabu_log_hex("qs:gen-pre=", rd(QSCRATCH_GENERAL_CFG));
  nabu_log_hex("qs:hs-pre=", rd(QSCRATCH_HS_PHY_CTRL));
  // dwc3_msm_qscratch_reg_init: RAM/BAM clock master enable.
  wr(QSCRATCH_CGCTL, rd(QSCRATCH_CGCTL) | 0x18);
  // POST_RESET half for an HS-only port: route the UTMI clock through the PIPE
  // and put PHYSTATUS under software control. The vendor also toggles
  // PIPE_UTMI_CLK_DIS around this, but that sequence locks the core on this
  // board (red line #4, hit twice), so the clock is never disabled here.
  wr(QSCRATCH_GENERAL_CFG, rd(QSCRATCH_GENERAL_CFG) |
     QSCRATCH_PIPE_UTMI_CLK_SEL | QSCRATCH_PIPE3_PHYSTATUS_SW);
  nabu_log_hex("qs:cg=", rd(QSCRATCH_CGCTL));
  nabu_log_hex("qs:gen=", rd(QSCRATCH_GENERAL_CFG));

  const uint32_t gctl0 = rd(DWC3_GCTL);
  wr(DWC3_GCTL, gctl0 | DWC3_GCTL_CORESOFTRESET);
  (void)waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, DWC3_GCTL_CORESOFTRESET, 10000);
  wr(DWC3_GCTL, gctl0 & ~DWC3_GCTL_CORESOFTRESET);
  if (!waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, 0, 10000)) {
    nabu_log_puts("dwc3:reset-stuck\n");
    return -1;
  }

  // PRTCAPDIR read back as 1 (HOST) even after a read-modify-write, so write the
  // target value outright instead of preserving whatever the reset left.
  wr(DWC3_GCTL, DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_DEVICE) | DWC3_GCTL_DSBLCLKGTNG);
  for (unsigned i = 0; i < 500000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  nabu_log_hex("dwc3:gsts2=", rd(DWC3_GSTS));
  // dwc3_gadget_init() resets the device controller before anything else.
  {
    uint32_t dctl = rd(DWC3_DCTL);
    wr(DWC3_DCTL, dctl | DWC3_DCTL_CSFTRST);
    (void)waitMask(g_regs + DWC3_DCTL / 4, DWC3_DCTL_CSFTRST, DWC3_DCTL_CSFTRST, 10000);
    wr(DWC3_DCTL, dctl & ~DWC3_DCTL_CSFTRST);
    (void)waitMask(g_regs + DWC3_DCTL / 4, DWC3_DCTL_CSFTRST, 0, 10000);
    nabu_log_hex("dwc3:dctl=", rd(DWC3_DCTL));
  }

  // dwc3_core_init() pulses GUSB2PHYCFG.PHYSOFTRST: the USB2 PHY interface needs
  // that soft reset before the controller will drive D+.
  {
    uint32_t p2 = rd(DWC3_GUSB2PHYCFG(0));
    nabu_log_hex("dwc3:g2cfg-pre=", p2);
    // dwc3_dis_sleep_mode() in the vendor glue: LPM is cleared before the gadget
    // is started. This board's bootloader leaves ENBLSLPM set.
    p2 &= ~DWC3_GUSB2PHYCFG_ENBLSLPM;
    wr(DWC3_GUSB2PHYCFG(0), p2 | DWC3_GUSB2PHYCFG_PHYSOFTRST);
    for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
    wr(DWC3_GUSB2PHYCFG(0), p2 & ~DWC3_GUSB2PHYCFG_PHYSOFTRST);
    for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
    nabu_log_hex("dwc3:g2rst=", rd(DWC3_GUSB2PHYCFG(0)));
  }

  // QMP (USB3) PHY is not brought up: suspend the SS pipe and ask for HS, or the
  // core negotiates a SuperSpeed link that has no lane and the host sees nothing.
  nabu_log_hex("dwc3:g3ctl=", rd(DWC3_GUSB3PIPECTL(0)));
  wr(DWC3_GUSB3PIPECTL(0), rd(DWC3_GUSB3PIPECTL(0)) | (1u << 17));
  nabu_log_hex("dwc3:g3ctl2=", rd(DWC3_GUSB3PIPECTL(0)));
  // core.h: DCFG_HIGHSPEED = (0 << 0), SUPERSPEED = (4 << 0). The DT's
  // maximum-speed = "high-speed" is the reset default, so clearing the field is
  // exactly right -- do not "fix" this to 1, that is FullSpeed.
  wr(DWC3_DCFG, rd(DWC3_DCFG) & ~0x7u);
  nabu_log_hex("dwc3:dcfg=", rd(DWC3_DCFG));
  const bool events = usbEventInit();
  // Land the ring before any endpoint command runs: a hang inside ep0Init must
  // still leave evidence on the medium instead of a silent boot followed by a
  // watchdog reset.  // Program EP0 and arm the SETUP TRB before the pull-up goes up: the host's
  // first SETUP must find a request waiting, or it times the transfer out and
  // suspends the port (run 54: U0 -> RxDetect -> U3, no SET_USB_RESET event).
  const bool ep0ok = events && ep0Init();
  logProbe("p:ep0\n");
  // Firmware already carries a valid SMMU entry for this stream (run 17 read
  // SMR=0x80000140 with S2CR=3), so the event ring can DMA without us taking a
  // context bank. With the DMA path accounted for, assert the software VBUS
  // override exactly where dwc3-msm does it: before starting the gadget.
  // dwc3_otg_start_peripheral() -> dwc3_override_vbus_status(true): HS_PHY_CTRL
  // bit 20. The bootloader leaves SW_SESSVLD_SEL=1 with UTMI_OTG_VBUS_VALID=0, so
  // the PHY takes session-valid from software and sees 0: no D+ pull-up, no host.
  // Run 68 is the proof -- qs:hs=10100000, bus reset, three SETUPs, 13 events. The
  // build that made this read-only (justified by the older AHB-lockup runs, which
  // predate the PHY sequence) is the one that went back to zero events.
  nabu_log_hex("qs:hs-pre2=", rd(QSCRATCH_HS_PHY_CTRL));
  wr(QSCRATCH_HS_PHY_CTRL, rd(QSCRATCH_HS_PHY_CTRL) | QSCRATCH_UTMI_OTG_VBUS_VALID);
  nabu_log_hex("qs:hs=", rd(QSCRATCH_HS_PHY_CTRL));
  nabu_log_hex("qs:osts-pre=", rd(DWC3_OSTS));  // The PMIC reports CDP, so run the vendor's DP pulse here -- before RUN_STOP,
  // and releasing every override when it ends. The previous order was the
  // reverse (force after RUN_STOP and hold), and the host never sent a reset.
  femtoDpPulse();
  // DCTL.RUN_STOP = 1 is run (dwc3_gadget_run_stop sets it when is_on). The
  // previous code cleared it immediately after setting it, which halts the core
  // and leaves D+ without a pull-up.
  {
    uint32_t dctl = rd(DWC3_DCTL);
    dctl |= DWC3_DCTL_RUN_STOP;
    dctl &= ~DWC3_DCTL_HIRD_THRES(0x1fu);
    dctl |= DWC3_DCTL_HIRD_THRES(0x10);          // DT: snps,hird-threshold = 0x10
    wr(DWC3_DCTL, dctl);
    nabu_log_hex("dwc3:dctl-post=", rd(DWC3_DCTL));
    nabu_log_hex("dwc3:dsts-run=", rd(DWC3_DSTS));
  }
  // Stop forcing the pull-up ourselves: the core is in U0 now, so if UTMI is
  // really working it drives TERMSEL/XCVRSEL by itself. Reading them back after
  // the run tells us whether the cores's UTMI side ever reaches this PHY.
  nabu_log_hex("phy:term-pre=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_hex("phy:xcvr-pre=", phyRd(FEMTO_CTRL1));
  // The host debounces the port for ~10 ms before it will reset it, and run 54
  // shows a 4M-yield spin covers only a few milliseconds (U0 -> RxDetect -> U3
  // happened inside one). So the answer window has to be two orders of magnitude
  // larger. 5000 yields between polls keeps a SETUP answer inside one host
  // frame. Run 64 measured the window: 24000 rounds lasted ~4 s of wall clock.
  // Run 68 pinned the host's timing down: it does nothing for the first ~3 s,
  // then RESET -> GET_DESCRIPTOR -> SET_ADDRESS (addr 0x36) -> GET_DESCRIPTOR,
  // all inside the last second of a 24000-round window. Runs 69 (flush at 4 s)
  // and 70 (flush at 3.5 s, none after) both wedged as soon as polling ran past
  // ~4 s, so the trigger is the work after that point, not the flush. The event
  // path now persists the ring tail (64 KiB, milliseconds) before every handler,
  // so the last thing the core showed us is on the medium even if the next
  // register access never returns.
  if (ep0ok) {
    const unsigned kInner = 5000;
    const unsigned kRounds = 60000;
    const unsigned kReplug = kRounds / 3;
    unsigned replugs = 0;
    for (unsigned round = 0; round < kRounds; ++round) {
      for (unsigned i = 0; i < kInner; ++i) { __asm__ volatile("yield" ::: "memory"); }
      usbPollEvents();
      // Ring only: the UFS writer thread publishes this off the control path,
      // every 2 s, so the enumeration loop never blocks on a disk write.
      if ((round % 1000) == 0) {
        nabu_log_hex("p:round=", round);
      }
      // Nothing has ever addressed us: the host sampled the port before the
      // pull-up existed, or gave up on its first attempt. Dropping and restoring
      // RUN_STOP makes it see a fresh connect. Vendor run_stop(1) redoes
      // start_config, so the endpoint program is repeated on every re-plug.
      if (replugs < 2 && round != 0 && (round % kReplug) == 0 && g_usbEvents == 0) {
        ++replugs;
        wr(DWC3_DCTL, rd(DWC3_DCTL) & ~DWC3_DCTL_RUN_STOP);
        for (unsigned i = 0; i < 200000; ++i) { __asm__ volatile("yield" ::: "memory"); }
        const uint32_t halted = rd(DWC3_DSTS) & DWC3_DSTS_DEVCTRLHLT;
        ep0Program();
        wr(DWC3_DCTL, rd(DWC3_DCTL) | DWC3_DCTL_RUN_STOP);
        nabu_log_hex("usb:replug=", static_cast<unsigned long long>(replugs) |
                     (static_cast<unsigned long long>(halted) << 32));
      }
      // Progress marker carrying the SOF frame number, so the log shows real
      // time too: frames ticking means the host has the port enabled.
      if (round != 0 && (round % 6000) == 0) {
        const uint32_t dsts = rd(DWC3_DSTS);
        nabu_log_hex("usb:alive=", static_cast<unsigned long long>(round) |
                     (static_cast<unsigned long long>(g_usbEvents & 0xffu) << 24) |
                     (static_cast<unsigned long long>(DWC3_DSTS_SOF_FN(dsts)) << 32));
        // DCFG rides along so the log shows whether the SET_ADDRESS write stuck
        // while the host is silent (a wrong DevAddr makes every later packet OOR).
        nabu_log_hex("usb:alive2=", dsts |
                     (static_cast<unsigned long long>(rd(DWC3_DCFG)) << 32));
      }
    }
    nabu_log_hex("usb:events=", g_usbEvents);
    nabu_log_hex("usb:setups=", g_ep0Setups);
    // End of the window: DCFG still holding DevAddr is what SET_ADDRESS was for.
    nabu_log_hex("usb:dcfg-end=", rd(DWC3_DCFG));
  } else if (events) {
    // EP0 never got a request, so nothing here can answer the host. Drain the
    // ring for the log and leave enumeration for a later run.
    for (unsigned round = 0; round < 200; ++round) {
      for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
      usbPollEvents();
    }
  }
  nabu_log_hex("qs:dsts2=", rd(DWC3_DSTS));
  nabu_log_hex("qs:evtcnt=", rd(DWC3_GEVNTCOUNT(0)));
  nabu_log_hex("phy:term2=", phyRd(FEMTO_UTMI_CTRL0));
  if (events) {
    nabu_log_hex("usb:va0=", g_evt[0]);
    // Same words read through a fresh mapping of the physical address the core
    // was given. If these differ, g_evt is not the memory the DMA targets.
    {
      OSSharedPtr<IOMemoryMap> paKeep;
      volatile uint32_t *pa = reinterpret_cast<volatile uint32_t *>(
        mapPhys(g_evtPhys, 0x1000, paKeep));
      if (pa == nullptr) {
        nabu_log_puts("usb:pa-map-fail\n");
      } else {
        nabu_log_hex("usb:pa0=", pa[0]);
        nabu_log_hex("usb:pa4=", pa[1]);
      }
    }
  }
  if (ep0ok) {
    nabu_log_hex("usb:trb0=", g_ep0[0]);
    nabu_log_hex("usb:trb0c=", g_ep0[3]);
    nabu_log_hex("usb:dalep=", rd(DWC3_DALEPENA));
    nabu_log_hex("usb:configured2=", static_cast<unsigned long long>(g_configured));
    nabu_log_hex("usb:last-link=", static_cast<unsigned long long>(g_last_link));
    usbPollEvents();
  }
  nabu_log_hex("phy:term-post=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_hex("phy:xcvr-post=", phyRd(FEMTO_CTRL1));
  nabu_log_hex("phy:cfg0-post=", phyRd(FEMTO_CFG0));
  nabu_log_hex("dwc3:dctl3=", rd(DWC3_DCTL));
  nabu_log_hex("dwc3:osts2=", rd(DWC3_OSTS));
  nabu_log_hex("dwc3:g2cfg2=", rd(DWC3_GUSB2PHYCFG(0)));
  nabu_log_hex("dwc3:gctl2=", rd(DWC3_GCTL));
  nabu_log_hex("dwc3:gsts=", rd(DWC3_GSTS));
  nabu_log_hex("dwc3:g2cfg=", rd(DWC3_GUSB2PHYCFG(0)));
  nabu_log_hex("dwc3:dsts=", rd(DWC3_DSTS));
  nabu_log_puts("dwc3:stage2-done\n");
  logProbe("p:stage2\n");
  return 2;
}

} // namespace

extern "C" int
native_dwc3_bringup(void)
{
  /* The watchdog thread in NativeUfs treats a frozen gDwc3Ticks as a hang and
   * resets the board, so the flag must be set before any core register is
   * touched and cleared on every return path. */
  gDwc3Window = 1;
  const int result = dwc3Bringup();
  gDwc3Window = 0;
  return result;
}

bool NativeDwc3::init(OSDictionary *dict)
{
  return super::init(dict);
}

void NativeDwc3::free(void)
{
  super::free();
}

IOService *NativeDwc3::probe(IOService *provider, SInt32 *score)
{
  return super::probe(provider, score);
}

bool NativeDwc3::start(IOService *provider)
{
  if (!super::start(provider)) { return false; }
  (void)dwc3Bringup();
  return true;
}

void NativeDwc3::stop(IOService *provider)
{
  g_regs = nullptr;
  g_keep.reset();
  g_gcc = nullptr;
  g_gccKeep.reset();
  super::stop(provider);
}

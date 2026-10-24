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
extern "C" int nabu_log_flush(int is_panic);

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

  // Vendor sets VREGBYPASS=1, i.e. the PHY expects 1.8 V on vdda18 and bypasses
  // its own regulator. The analog side never responds here, so try the internal
  // regulator instead: if vdda18 is the missing rail, this is what brings the
  // D+ pull-up back.
  phyWrMask(FEMTO_COMMON2, 1u << 0, 0);           // VREGBYPASS = 0
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
  nabu_log_puts("phy:init-done\n");
  return true;
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

static bool usbEventInit(void)
{
  nabu_log_puts("usb:alloc\n");
  nabu_log_flush(0);
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
  nabu_log_flush(0);
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
  nabu_log_flush(0);
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
  nabu_log_flush(0);

  wr(DWC3_GEVNTADRLO(0), static_cast<uint32_t>(g_evtPhys));
  wr(DWC3_GEVNTADRHI(0), static_cast<uint32_t>(g_evtPhys >> 32));
  wr(DWC3_GEVNTSIZ(0), 4096);
  wr(DWC3_GEVTEN, 1);
  wr(DWC3_DEVTEN, 0xf); // disconnect, reset, connect-done, setup
  nabu_log_hex("usb:evtadr=", rd(DWC3_GEVNTADRLO(0)));
  nabu_log_hex("usb:evtadrhi=", rd(DWC3_GEVNTADRHI(0)));
  nabu_log_hex("usb:evtsiz=", rd(DWC3_GEVNTSIZ(0)));
  nabu_log_hex("usb:evtcnt=", rd(DWC3_GEVNTCOUNT(0)));
  // A hang right after the ring is armed loses everything still in DRAM, so put
  // the arm step on the medium before the risky part starts.
  nabu_log_flush(0);
  return true;
}

static void usbPollEvents(void)
{
  uint32_t count = rd(DWC3_GEVNTCOUNT(0));
  if (count == 0) {
    return;
  }
  nabu_log_hex("usb:evtcnt2=", count);
  unsigned offset = 0;
  while (offset + 4 <= count) {
    // DWC3 event TRBs: type lives in bits[7:1] and a device event is 4 bytes.
    // The old decode (type = word & 0xff, offset += 8) read the wrong fields and
    // skipped half the ring, so every event looked like type 0 / devt 0.
    uint32_t ev = g_evt[offset / 4];
    uint32_t type = (ev >> 1) & 0x3fu;
    nabu_log_hex("usb:ev0=", ev);
    if (type == 0) {                        // device event, 4 bytes
      nabu_log_hex("usb:devt=", (ev >> 9) & 0x7fu);   // 1 disc, 2 conn-done, 3 link
      nabu_log_hex("usb:link=", (ev >> 17) & 0xfu);
      offset += 4;
    } else if (type == 6) {                 // endpoint event (+8 if SETUP)
      nabu_log_hex("usb:ep1=", g_evt[offset / 4 + 1]);
      nabu_log_hex("usb:ep2=", g_evt[offset / 4 + 2]);
      nabu_log_hex("usb:ep3=", g_evt[offset / 4 + 3]);
      offset += 12;
    } else {
      offset += 4;
    }
  }
  wr(DWC3_GEVNTCOUNT(0), count & 0xfffcu);
}

static int dwc3Bringup(void)
{
  if (g_bringup_done) {
    return 0;
  }
  g_bringup_done = 1;
  nabu_log_puts("dwc3:start\n");
  nabu_log_puts("usb:marker\n");

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
    nabu_log_flush(0);
    return 1;
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
      static const unsigned kSpmiRegs[] = { 0x1010, 0x1008, 0x1700, 0x1704 };
      for (unsigned r = 0; r < sizeof(kSpmiRegs) / sizeof(kSpmiRegs[0]); ++r) {
        const unsigned addr = kSpmiRegs[r];
        const unsigned ppid = (2u << 8) | ((addr >> 8) & 0xffu);  // PM8150B = sid 2
        unsigned apid = 0xffffffffU;
        // The APID table is only 0x400 bytes (256 entries) from 0x900; reading
        // past it hits unimplemented addresses and wedges the arbiter's AXI
        // slave, which is what killed run 20.
        for (unsigned a = 0; a < 128; ++a) {
          uint32_t ent = spmi[(0x900 / 4) + a];
          if (!ent) { continue; }
          if (((ent >> 8) & 0xfffu) == ppid) { apid = a; break; }
        }
        nabu_log_hex("spmi:ppid=", ppid);
        nabu_log_hex("spmi:apid=", apid == 0xffffffffU ? 0xfffu : apid);
        if (apid == 0xffffffffU) {
          continue;
        }
        // v5 observer channel: 0x10000*ee + 0x80*apid. Map the containing page
        // and offset into it, so an unaligned 0x80*apid still maps.
        OSSharedPtr<IOMemoryMap> chanKeep;
        const uint64_t chanPhys = 0x0C440000ULL + 0x80ULL * apid;
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
    }
  }
  (void)rpmhUsbPhyRails();
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

  // dwc3_msm_qscratch_reg_init: RAM/BAM clock master enable.
  wr(QSCRATCH_CGCTL, rd(QSCRATCH_CGCTL) | 0x18);
  // POST_RESET half for an HS-only port: route the UTMI clock through the PIPE
  // and put PHYSTATUS under software control. The vendor also toggles
  // PIPE_UTMI_CLK_DIS around this, but doing that to a running core wedged the
  // AHB slave on this board (run 3), so the clock is never disabled here.
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
  // Firmware already carries a valid SMMU entry for this stream (run 17 read
  // SMR=0x80000140 with S2CR=3), so the event ring can DMA without us taking a
  // context bank. With the DMA path accounted for, assert the software VBUS
  // override exactly where dwc3-msm does it: before starting the gadget.
  wr(QSCRATCH_HS_PHY_CTRL, rd(QSCRATCH_HS_PHY_CTRL) | QSCRATCH_UTMI_OTG_VBUS_VALID);
  nabu_log_hex("qs:hs=", rd(QSCRATCH_HS_PHY_CTRL));
  nabu_log_hex("qs:osts-pre=", rd(DWC3_OSTS));
  nabu_log_flush(0);
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
    nabu_log_flush(0);
  }
  // Stop forcing the pull-up ourselves: the core is in U0 now, so if UTMI is
  // really working it drives TERMSEL/XCVRSEL by itself. Reading them back after
  // the run tells us whether the cores's UTMI side ever reaches this PHY.
  nabu_log_hex("phy:term-pre=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_hex("phy:xcvr-pre=", phyRd(FEMTO_CTRL1));
  nabu_log_flush(0);
  // Give the host time to enumerate while the pull-up is held.
  for (unsigned i = 0; i < 4000000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  nabu_log_hex("qs:dsts2=", rd(DWC3_DSTS));
  nabu_log_hex("qs:evtcnt=", rd(DWC3_GEVNTCOUNT(0)));
  nabu_log_hex("phy:term2=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_flush(0);
  // Force the PHY's own pull-up now that the core is already in U0. These are
  // the registers drive_dp_pulse() toggles; holding them on (instead of pulsing)
  // is the one combination never tried together with the VBUS override, the S2CR
  // bypass and a readable event ring.
  phyWrMask(FEMTO_CFG0, 3u, 3u);                  // CMN + DATAPATH override
  phyWrMask(FEMTO_UTMI_CTRL0, 0x18, 0x00);        // OPMODE = NORMAL
  phyWrMask(FEMTO_UTMI_CTRL0, 1u << 5, 1u << 5);  // TERMSEL
  phyWrMask(FEMTO_CTRL1, 1u << 0, 1u << 0);       // XCVRSEL
  nabu_log_hex("phy:term-force=", phyRd(FEMTO_UTMI_CTRL0));
  nabu_log_hex("phy:xcvr-force=", phyRd(FEMTO_CTRL1));
  nabu_log_flush(0);
  if (events) {
    // No long holds of any kind: every attempt to keep the link up longer
    // (IODelay, CNTPCT spin) has ended in a wedged boot. Just drain what the
    // ring already has and let the kernel continue.
    for (unsigned round = 0; round < 200; ++round) {
      for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
      usbPollEvents();
    }
    nabu_log_hex("dwc3:dsts2=", rd(DWC3_DSTS));
    nabu_log_hex("usb:va0=", g_evt[0]);
    nabu_log_hex("phy:term-post=", phyRd(FEMTO_UTMI_CTRL0));
    nabu_log_hex("phy:xcvr-post=", phyRd(FEMTO_CTRL1));
    nabu_log_hex("phy:cfg0-post=", phyRd(FEMTO_CFG0));
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
    nabu_log_hex("dwc3:dctl3=", rd(DWC3_DCTL));
    nabu_log_hex("dwc3:osts2=", rd(DWC3_OSTS));
    nabu_log_hex("dwc3:g2cfg2=", rd(DWC3_GUSB2PHYCFG(0)));
    usbPollEvents();
  }
  nabu_log_hex("dwc3:gctl2=", rd(DWC3_GCTL));
  nabu_log_hex("dwc3:gsts=", rd(DWC3_GSTS));
  nabu_log_hex("dwc3:g2cfg=", rd(DWC3_GUSB2PHYCFG(0)));
  nabu_log_hex("dwc3:dsts=", rd(DWC3_DSTS));
  nabu_log_puts("dwc3:stage2-done\n");
  return 2;
}

} // namespace

extern "C" int
native_dwc3_bringup(void)
{
  return dwc3Bringup();
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

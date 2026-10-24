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

  // usb_1 glue names GCC_USB3_SEC_CLKREF_CLK, not the PRIM one.
  gwr(GCC_USB3_SEC_CLKREF_CLK, grd(GCC_USB3_SEC_CLKREF_CLK) | GCC_BRANCH_ENABLE_BIT);

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
#define FEMTO_UTMI_CTRL5 0x50
#define FEMTO_COMMON0 0x54
#define FEMTO_COMMON1 0x58
#define FEMTO_COMMON2 0x5c
#define FEMTO_CTRL1 0x60
#define FEMTO_CTRL2 0x64
#define FEMTO_CFG0 0x94
#define FEMTO_REFCLK 0xa0

static OSSharedPtr<IOMemoryMap> g_phyKeep;
static volatile uint32_t *g_phy;

static uint32_t phyRd(unsigned long off) { return g_phy[off / 4]; }
static void phyWr(unsigned long off, uint32_t v) { g_phy[off / 4] = v; }
static void phyWrMask(unsigned long off, uint32_t mask, uint32_t val)
{
  uint32_t v = phyRd(off);
  phyWr(off, (v & ~mask) | (val & mask));
}

static bool femtoPhyInit(void)
{
  const uintptr_t va = mapPhys(QUSB2_BASE, 0x1000, g_phyKeep);
  g_phy = reinterpret_cast<volatile uint32_t *>(va);
  if (g_phy == nullptr) {
    return false;
  }
  nabu_log_hex("phy:r0=", phyRd(0x0));
  nabu_log_hex("phy:r4=", phyRd(0x4));
  nabu_log_hex("phy:r1a0=", phyRd(0x1a0));
  nabu_log_hex("phy:r800=", phyRd(0x800));
  nabu_log_hex("phy:common2=", phyRd(FEMTO_COMMON2));
  nabu_log_hex("phy:cfg0=", phyRd(FEMTO_CFG0));
  // CFG0 reads 0 on a healthy but uninitialised femto PHY; COMMON2 (0x5c)
  // returning 0xc1c1c1c1 is its reset value. Do not bail out here.
  phyWrMask(FEMTO_CFG0, 1u << 1, 1u << 1);        // CMN_CTRL_OVERRIDE_EN
  phyWrMask(FEMTO_UTMI_CTRL5, 1u << 1, 1u << 1);  // POR
  phyWrMask(FEMTO_COMMON0, 0x70, 0);              // FSEL = 0
  phyWrMask(FEMTO_COMMON1, 1u << 5, 1u << 5);     // PLLBTUNE
  phyWrMask(FEMTO_REFCLK, 0x3, 0x2);              // REFCLK_SEL_DEFAULT
  phyWrMask(FEMTO_COMMON1, 1u << 4, 1u << 4);     // VBUSVLDEXTSEL0
  phyWrMask(FEMTO_CTRL1, 1u << 0, 1u << 0);       // VBUSVLDEXT0
  phyWrMask(FEMTO_COMMON2, 1u << 0, 1u << 0);     // VREGBYPASS
  phyWrMask(FEMTO_CTRL2, (1u << 3) | (1u << 2), (1u << 3) | (1u << 2));
  phyWrMask(FEMTO_UTMI_CTRL0, 1u << 0, 1u << 0);  // SLEEPM
  phyWrMask(FEMTO_COMMON0, 1u << 2, 0);           // SIDDQ = 0
  phyWrMask(FEMTO_UTMI_CTRL5, 1u << 1, 0);        // POR = 0
  phyWrMask(FEMTO_CTRL2, 1u << 3, 0);             // SUSPEND_N_SEL = 0
  phyWrMask(FEMTO_CFG0, 1u << 1, 0);              // CMN override off

  for (unsigned i = 0; i < 100000; ++i) {
    __asm__ volatile("yield" ::: "memory");
  }
  nabu_log_hex("phy:cfg0b=", phyRd(FEMTO_CFG0));
  nabu_log_hex("phy:common0=", phyRd(FEMTO_COMMON0));
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
  g_evt = reinterpret_cast<volatile uint32_t *>(g_evtDma->getBytesNoCopy());
  IOByteCount contiguous = 0;
  g_evtPhys = g_evtDma->getPhysicalSegment(0, &contiguous, kIOMemoryMapperNone);
  nabu_log_hex("usb:evtpa=", g_evtPhys);
  nabu_log_hex("usb:evtlen=", contiguous);
  if (g_evt == nullptr || g_evtPhys == 0 || (g_evtPhys & 4095) || contiguous < 4096) {
    nabu_log_puts("usb:dma-bad\n");
    return false;
  }
  for (unsigned i = 0; i < 1024; ++i) { g_evt[i] = 0; }
  nabu_log_puts("usb:smmu\n");
  nabu_log_flush(0);
  // Keep the SMMU untouched for now: bypassing 0x140 shares UFS context banks.
  // (void)native_smmu_bypass(0x140, g_evtPhys, 4096);
  nabu_log_puts("usb:smmu-done\n");
  nabu_log_flush(0);

  wr(DWC3_GEVNTADRLO(0), static_cast<uint32_t>(g_evtPhys));
  wr(DWC3_GEVNTADRHI(0), static_cast<uint32_t>(g_evtPhys >> 32));
  wr(DWC3_GEVNTSIZ(0), 4096);
  wr(DWC3_GEVTEN, 1);
  wr(DWC3_DEVTEN, 0xf); // disconnect, reset, connect-done, setup
  nabu_log_hex("usb:evtcnt=", rd(DWC3_GEVNTCOUNT(0)));
  return true;
}

static void usbPollEvents(void)
{
  uint32_t count = rd(DWC3_GEVNTCOUNT(0));
  nabu_log_hex("usb:evtcnt2=", count);
  if (count == 0) {
    return;
  }
  nabu_log_hex("usb:evt0=", g_evt[0]);
  nabu_log_hex("usb:evt1=", g_evt[1]);
  nabu_log_hex("usb:evt2=", g_evt[2]);
  wr(DWC3_GEVNTCOUNT(0), count & 0xfffc);
}

static int dwc3Bringup(void)
{
  if (g_bringup_done) {
    return 0;
  }
  g_bringup_done = 1;
  nabu_log_puts("dwc3:start\n");
  nabu_log_puts("usb:marker\n");

  int enable = 1, init = 1;
  (void)PE_parse_boot_argn("nabu-usb", &enable, sizeof(enable));
  (void)PE_parse_boot_argn("nabu-usb-init", &init, sizeof(init));
  if (!enable) {
    nabu_log_puts("dwc3:off\n");
    return 0;
  }

  rpmhProbe();
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

  // Release the QUSB2 PHY reset before the femto init, as qcom_snps_hsphy_init does.
  if (g_gcc != nullptr) {
    gwr(GCC_QUSB2PHY_PRIM_BCR, 0x1);
    for (unsigned i = 0; i < 20000; ++i) { __asm__ volatile("yield" ::: "memory"); }
    gwr(GCC_QUSB2PHY_PRIM_BCR, 0x0);
  }

  nabu_log_hex("dwc3:bcr=", grd(GCC_USB30_PRIM_BCR));
  nabu_log_hex("dwc3:ahbbcr=", grd(GCC_USB_PHY_CFG_AHB2PHY_BCR));
  nabu_log_hex("dwc3:ahbclk=", grd(0x6a004));
  nabu_log_hex("dwc3:gdscr2=", grd(GCC_USB30_PRIM_GDSCR));
  nabu_log_hex("dwc3:phybcr=", grd(GCC_QUSB2PHY_PRIM_BCR));
  if (!femtoPhyInit()) {
    nabu_log_puts("dwc3:phy-bad\n");
  }


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

  const uint32_t gctl0 = rd(DWC3_GCTL);
  wr(DWC3_GCTL, gctl0 | DWC3_GCTL_CORESOFTRESET);
  (void)waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, DWC3_GCTL_CORESOFTRESET, 10000);
  wr(DWC3_GCTL, gctl0 & ~DWC3_GCTL_CORESOFTRESET);
  if (!waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, 0, 10000)) {
    nabu_log_puts("dwc3:reset-stuck\n");
    return -1;
  }

  uint32_t gctl = rd(DWC3_GCTL);
  gctl &= ~DWC3_GCTL_SCALEDOWN(3);
  gctl |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_DEVICE);
  gctl |= DWC3_GCTL_DSBLCLKGTNG;
  wr(DWC3_GCTL, gctl);

  uint32_t p2 = rd(DWC3_GUSB2PHYCFG(0));
  p2 &= ~DWC3_GUSB2PHYCFG_SUSPHY;
  wr(DWC3_GUSB2PHYCFG(0), p2);

  // DEVTEN first, then start the controller: the connect event is generated by the
  // RUN_STOP transition and is lost if the event mask is still closed.
  const bool events = usbEventInit();
  wr(DWC3_DCTL, rd(DWC3_DCTL) & ~DWC3_DCTL_RUN_STOP);
  if (events) {
    for (unsigned i = 0; i < 500000; ++i) { __asm__ volatile("yield" ::: "memory"); }
    usbPollEvents();
  }
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

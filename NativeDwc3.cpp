// NativeDwc3 -- SM8150 USB0 (DWC3) bring-up, staged so a hang localises itself.
//
// Stage 0  GCC: power usb30_prim GDSC, enable clkref + the branch clocks and the
//          QUSB2 PHY reset deassert. Nothing here touches the controller.
// Stage 1  read-only core identification (GSNPSID, GHWPARAMS0, DSTS).
// Stage 2  (gated by nabu-usb-init=1) core soft reset, device mode, RUN_STOP.
//
// Entry point is native_dwc3_bringup(), called by the platform probe. The
// IOService personality cannot win the single IOPlatformDevice nub (NativeUfs
// scores 1000), so start() is not relied on.
//
// Everything is behind nabu-usb=1 in boot-args: the driver can ship in the kernel
// while the board keeps booting with USB untouched.
//
// Follows Android drivers/usb/dwc3/core.c (dwc3_core_init) and
// drivers/usb/dwc3/dwc3-qcom.c (clock/GDSC order). See DESIGN.md.

#include "NativeDwc3.h"
#include "Dwc3Registers.h"
#include "GccUsb.h"

#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSSharedPtr.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>

extern "C" void nabu_mark_text(const char *s, unsigned int y);
extern "C" void nabu_mark_num (const char *label, unsigned long v, unsigned int y);
/* The host reads the UFS log window, so boot readings must go to the ring too:
 * IOLog no longer reaches it once the console leaves printbuf. */
extern "C" void nabu_log_puts(const char *s);
extern "C" void nabu_log_hex(const char *label, unsigned long long value);

#define super IOService
OSDefineMetaClassAndStructors(NativeDwc3, IOService)

namespace {

OSSharedPtr<IOMemoryMap> g_keep;
OSSharedPtr<IOMemoryMap> g_gccKeep;
volatile uint32_t *g_regs = nullptr;
volatile uint32_t *g_gcc = nullptr;
int g_bringup_done = 0;
int g_bringup_result = -1;

inline uint32_t rd(unsigned long off) { return g_regs[off / 4]; }
inline void wr(unsigned long off, uint32_t v) { g_regs[off / 4] = v; }
inline uint32_t grd(unsigned long off) { return g_gcc[off / 4]; }
inline void gwr(unsigned long off, uint32_t v) { g_gcc[off / 4] = v; }

// same pattern NativeUfs uses; proved to work on this platform
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

// usb30_prim GDSC on, then every branch the controller needs. Returns how many
// branch clocks failed to report CLK_OFF = 0 (0 is the healthy result).
static unsigned gccUsbPowerOn(void)
{
  const uintptr_t va = mapPhys(GCC_BASE, 0x100000, g_gccKeep);
  g_gcc = reinterpret_cast<volatile uint32_t *>(va);
  nabu_mark_num("dwc3:gcc=", (unsigned long)va, 2100);
  if (g_gcc == nullptr) {
    return 0xff;
  }

  // Power domain first: the branch registers do not stick while it is collapsed.
  uint32_t gdscr = grd(GCC_USB30_PRIM_GDSCR);
  nabu_log_hex("dwc3:gdscr=", gdscr);
  gwr(GCC_USB30_PRIM_GDSCR, gdscr & ~GCC_GDSCR_SW_COLLAPSE);
  if (!waitMask(g_gcc + GCC_USB30_PRIM_GDSCR / 4, GCC_GDSCR_PWR_ON, GCC_GDSCR_PWR_ON, 100000)) {
    nabu_log_puts("dwc3:gdsc-timeout\n");
    return 0xfe;
  }

  // 19.2 MHz reference to the PHY PLL. The usb_1 glue node names
  // GCC_USB3_SEC_CLKREF_CLK (0x8c028), not the PRIM one.
  gwr(GCC_USB3_SEC_CLKREF_CLK, grd(GCC_USB3_SEC_CLKREF_CLK) | GCC_BRANCH_ENABLE_BIT);

  // Master source: 200 MHz = GPLL0_OUT_MAIN / 3. Only configure it when the
  // register reads back something that is not already that setting -- boot may
  // have left it valid and rewriting a live RCG can glitch the branch.
  const uint32_t cfg = grd(GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CFG);
  nabu_log_hex("dwc3:rcgcfg=", cfg);
  /* Boot leaves MODE=1 (MND) with M/N/D = 0, i.e. no output at all: the core
   * then reads back 0 from every register. Program DIV mode (200 MHz =
   * GPLL0_OUT_MAIN/3) with the master branch halted first -- an RCG write while
   * the branch runs is dropped. */
  nabu_log_hex("dwc3:rcgm=", grd(GCC_USB30_PRIM_MASTER_CLK_SRC + 0x8));
  gwr(GCC_USB30_PRIM_MASTER_CLK, grd(GCC_USB30_PRIM_MASTER_CLK) & ~GCC_BRANCH_ENABLE_BIT);
  (void)waitMask(g_gcc + GCC_USB30_PRIM_MASTER_CLK / 4, GCC_BRANCH_CLK_OFF, GCC_BRANCH_CLK_OFF, 100000);
  const uint32_t want = (GCC_USB_MASTER_SRC_GPLL0_MAIN << 8) | (GCC_USB_MASTER_DIV_200MHZ << 16);
  gwr(GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CFG, want);
  gwr(GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CMD, GCC_USB_MASTER_SRC_GPLL0_MAIN | GCC_RCG2_CMD_UPDATE);
  (void)waitMask(g_gcc + (GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CMD) / 4, GCC_RCG2_CMD_UPDATE, 0, 100000);
  nabu_log_hex("dwc3:rcgcfg2=", grd(GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CFG));

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

  // Release the controller reset (usb_1 glue: resets = <&gcc GCC_USB30_PRIM_BCR>).
  // Without this the core answers reads with bus default data, not GSNPSID.
  gwr(GCC_USB30_PRIM_BCR, 0x1);
  (void)waitMask(g_gcc + GCC_USB30_PRIM_BCR / 4, 0x1, 0, 100000);

  // Release the QUSB2 PHY from reset.
  gwr(GCC_QUSB2PHY_PRIM_BCR, 0x1);
  (void)waitMask(g_gcc + GCC_QUSB2PHY_PRIM_BCR / 4, 0x1, 0, 100000);
  return failed;
}

// ---- QUSB2 PHY (hsphy@88e2000) ------------------------------------------------
// v2 register layout and init table from drivers/phy/qualcomm/phy-qcom-qusb2.c.
// The DWC3 global registers only answer once this PHY supplies the UTMI clock;
// before that every core read returns 0 (hardware 2026-10-24).
#define QUSB2_BASE 0x088E2000ULL
#define QUSB2_PLL_STATUS 0x1a0
#define QUSB2_PLL_LOCKED (1u << 5)
#define QUSB2_PORT_POWERDOWN 0x210
#define QUSB2_POWER_DOWN (1u << 0)
#define QUSB2_CLAMP_N_EN (1u << 5)

static OSSharedPtr<IOMemoryMap> g_phyKeep;
static volatile uint32_t *g_phy;

static void phyWr(unsigned long off, uint32_t v) { g_phy[off / 4] = v; }
static uint32_t phyRd(unsigned long off) { return g_phy[off / 4]; }

static bool qusb2PhyInit(void)
{
  const uintptr_t va = mapPhys(QUSB2_BASE, 0x1000, g_phyKeep);
  g_phy = reinterpret_cast<volatile uint32_t *>(va);
  if (g_phy == nullptr) {
    return false;
  }
  static const unsigned long tbl[][2] = {
    { 0x004, 0x03 },  // PLL_ANALOG_CONTROLS_TWO
    { 0x18c, 0x7c },  // PLL_CLOCK_INVERTERS
    { 0x02c, 0x80 },  // PLL_CMODE
    { 0x184, 0x0a },  // PLL_LOCK_DELAY
    { 0x0b4, 0x19 },  // PLL_DIGITAL_TIMERS_TWO
    { 0x194, 0x40 },  // PLL_BIAS_CONTROL_1
    { 0x198, 0x20 },  // PLL_BIAS_CONTROL_2
    { 0x214, 0x21 },  // PWR_CTRL2
    { 0x220, 0x00 },  // IMP_CTRL1
    { 0x224, 0x58 },  // IMP_CTRL2
    { 0x240, 0x30 },  // TUNE1
    { 0x244, 0x29 },  // TUNE2: eFuse trim; 0x29 is the v2 default
    { 0x248, 0xca },  // TUNE3
    { 0x24c, 0x04 },  // TUNE4
    { 0x250, 0x03 },  // TUNE5
    { 0x23c, 0x00 },  // CHG_CTRL2
  };
  for (unsigned i = 0; i < sizeof(tbl) / sizeof(tbl[0]); ++i) {
    phyWr(tbl[i][0], static_cast<uint32_t>(tbl[i][1]));
  }
  uint32_t pd = phyRd(QUSB2_PORT_POWERDOWN);
  phyWr(QUSB2_PORT_POWERDOWN, pd & ~(QUSB2_POWER_DOWN | QUSB2_CLAMP_N_EN));
  for (unsigned i = 0; i < 1000000; ++i) {
    if (phyRd(QUSB2_PLL_STATUS) & QUSB2_PLL_LOCKED) {
      nabu_log_puts("dwc3:phy-lock\n");
      return true;
    }
    __asm__ volatile("yield" ::: "memory");
  }
  nabu_log_hex("dwc3:phy-status=", phyRd(QUSB2_PLL_STATUS));
  return false;
}

// ---- RPMh / cmd_db -------------------------------------------------------------
// Reference: drivers/soc/qcom/cmd-db.c, rpmh-rsc.c, qcom-rpmh-regulator.c.
// cmd_db is the aop_cmd_db reserved region (SM8150 DTS 0x85f20000);
// the AP's RSC is drv-2 at 0x18220000 with TCS at +0xd00.
#define CMD_DB_BASE 0x85F20000ULL
#define CMD_DB_SIZE 0x20000ULL
#define RSC_DRV2_BASE 0x18220000ULL
#define RSC_DRV_SIZE 0x10000ULL
#define RSC_TCS_OFFSET 0xd00

static OSSharedPtr<IOMemoryMap> g_cmdKeep, g_rscKeep;
static volatile uint8_t *g_cmd;
static volatile uint32_t *g_rsc;

static uint16_t cmdRd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t cmdRd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint32_t rscRd(unsigned long off) { return g_rsc[off / 4]; }

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

// Read-only: prove the table and the RSC are where the DTS says before any write.
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
  nabu_log_hex("rpmh:tcs_msgid=", rscRd(RSC_TCS_OFFSET + RSC_TCS_CMD_MSGID));
  nabu_log_hex("rpmh:tcs_addr=", rscRd(RSC_TCS_OFFSET + RSC_TCS_CMD_ADDR));
}

// ---- RPMh write path ------------------------------------------------------------
// v2.7 TCS layout (major = 1 < 3). One command per trigger, each waited for.
#define RSC_TCS_SIZE 672
#define RSC_TCS_CONTROL 0x14
#define RSC_TCS_CMD_ENABLE 0x1c
#define RSC_TCS_CMD_MSGID 0x30
#define RSC_TCS_CMD_ADDR 0x34
#define RSC_TCS_CMD_DATA 0x38
#define RSC_TCS_CMD_STATUS 0x3c
#define RSC_TCS_CMD_STEP 20
#define TCS_AMC_ENABLE (1u << 16)
#define TCS_AMC_TRIGGER (1u << 24)
#define RPMH_MSGID_LEN 8u
#define RPMH_MSGID_RESP (1u << 8)
#define RPMH_MSGID_WRITE (1u << 16)
#define RPMH_CMD_COMPL (1u << 16)

static void rscWr(unsigned long off, uint32_t v) { g_rsc[off / 4] = v; }

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

static int g_rpmhWriteEnable;

static bool rpmhUsbPhyRails(void)
{
  const uint32_t l2 = rpmhCmdDbAddr("ldoa2");
  const uint32_t l5 = rpmhCmdDbAddr("ldoa5");
  const uint32_t l12 = rpmhCmdDbAddr("ldoa12");
  nabu_log_hex("rpmh:l2=", l2);
  nabu_log_hex("rpmh:l5=", l5);
  nabu_log_hex("rpmh:l12=", l12);
  bool ok = true;
  ok = rpmhSetLdo(l5, 880) && ok;   // vdda-pll
  ok = rpmhSetLdo(l2, 3072) && ok;  // vdda33
  ok = rpmhSetLdo(l12, 1800) && ok; // vdda18
  nabu_log_puts(ok ? "rpmh:rails-ok\n" : "rpmh:rails-fail\n");
  return ok;
}

static int dwc3Bringup(void)
{
  if (g_bringup_done) {
    return g_bringup_result;
  }
  g_bringup_done = 1;
  g_bringup_result = 0;
  nabu_log_puts("dwc3:start\n");

  /* Off by default: stage 0 works (GDSC on, all branch clocks running) but the
   * core still reads 0 because the 7nm HS PHY needs PMIC LDOs XNU does not drive
   * yet. Flip nabu-usb=1 once that exists. */
  int enable = 1, init = 0;
  (void)PE_parse_boot_argn("nabu-usb", &enable, sizeof(enable));
  (void)PE_parse_boot_argn("nabu-usb-init", &init, sizeof(init));
  if (!enable) {
    nabu_log_puts("dwc3:off\n");
    nabu_mark_text("dwc3:off", 2100);
    return 0;
  }

  rpmhProbe();
  if (g_rpmhWriteEnable) { (void)rpmhUsbPhyRails(); }
  const unsigned failed = gccUsbPowerOn();
  if (!qusb2PhyInit()) {
    nabu_log_puts("dwc3:phy-bad\n");
  }
  nabu_log_hex("dwc3:clkfail=", failed);
  nabu_mark_num("dwc3:clkfail=", (unsigned long)failed, 2180);
  if (failed == 0xff) {
    nabu_log_puts("dwc3:gcc-map-bad\n");
    nabu_mark_text("dwc3:gcc-map-bad", 2100);
    return -1;
  }
  if (failed) {
    nabu_mark_text("dwc3:clk-bad", 2100);
    // Keep going: the core read below says whether the domain is actually usable.
  }

  // ---- stage 1: identify the core (read-only) ------------------------------
  const uintptr_t va = mapPhys(NABU_USB0_CORE_BASE, NABU_USB0_CORE_SIZE, g_keep);
  g_regs = reinterpret_cast<volatile uint32_t *>(va);
  nabu_mark_num("dwc3:core=", (unsigned long)va, 2260);
  if (g_regs == nullptr) {
    nabu_log_puts("dwc3:map-failed\n");
    nabu_mark_text("dwc3:map-failed", 2100);
    return -1;
  }

  nabu_log_hex("dwc3:cap=", rd(0x0));
  nabu_log_hex("dwc3:hcsp=", rd(0x4));
  nabu_log_hex("dwc3:gsbus=", rd(DWC3_GSBUSCFG0));
  nabu_log_hex("dwc3:guid=", rd(DWC3_GUID));
  const uint32_t sid = rd(DWC3_GSNPSID);
  nabu_log_hex("dwc3:sid=", sid);
  nabu_mark_num("dwc3:sid=", (unsigned long)sid, 2260);
  const uint32_t family = sid & DWC3_GSNPSID_MASK;
  if (family != DWC3_GSNPSID_DWC_USB3 && family != DWC3_GSNPSID_DWC_USB31) {
    nabu_log_puts("dwc3:id-bad\n");
    nabu_mark_text("dwc3:id-bad", 2100);
    return -1;
  }
  nabu_log_hex("dwc3:params0=", rd(DWC3_GHWPARAMS0));
  nabu_log_hex("dwc3:params1=", rd(DWC3_GHWPARAMS1));
  nabu_log_hex("dwc3:params3=", rd(DWC3_GHWPARAMS3));
  nabu_mark_num("dwc3:params=", (unsigned long)rd(DWC3_GHWPARAMS0), 2340);

  // ---- stage 2: controller init (only when asked) --------------------------
  if (!init) {
    nabu_log_puts("dwc3:stage1-done\n");
    nabu_mark_text("dwc3:stage1-done", 2100);
    return 1;
  }

  // dwc3_core_init(): pulse GCTL.CORESOFTRESET, wait for it to clear.
  const uint32_t gctl0 = rd(DWC3_GCTL);
  wr(DWC3_GCTL, gctl0 | DWC3_GCTL_CORESOFTRESET);
  if (!waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, DWC3_GCTL_CORESOFTRESET, 10000)) {
    nabu_mark_text("dwc3:reset-bad", 2100);
  }
  wr(DWC3_GCTL, gctl0 & ~DWC3_GCTL_CORESOFTRESET);
  if (!waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, 0, 10000)) {
    nabu_log_puts("dwc3:reset-stuck\n");
    nabu_mark_text("dwc3:reset-stuck", 2100);
    return -1;
  }

  // Device mode.
  uint32_t gctl = rd(DWC3_GCTL);
  gctl &= ~DWC3_GCTL_SCALEDOWN(3);
  gctl |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_DEVICE);
  gctl |= DWC3_GCTL_DSBLCLKGTNG;
  wr(DWC3_GCTL, gctl);

  uint32_t p2 = rd(DWC3_GUSB2PHYCFG(0));
  p2 &= ~DWC3_GUSB2PHYCFG_SUSPHY;
  wr(DWC3_GUSB2PHYCFG(0), p2);

  wr(DWC3_DCTL, rd(DWC3_DCTL) | DWC3_DCTL_RUN_STOP);
  nabu_log_hex("dwc3:dsts=", rd(DWC3_DSTS));
  nabu_mark_num("dwc3:dsts=", (unsigned long)rd(DWC3_DSTS), 2340);
  nabu_log_puts("dwc3:stage2-done\n");
  nabu_mark_text("dwc3:stage2-done", 2100);
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

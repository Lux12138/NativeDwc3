// NativeDwc3 -- SM8150 USB0 (DWC3) bring-up, staged so a hang localises itself.
//
// Stage 0  GCC: power usb30_prim GDSC, enable clkref + the branch clocks and the
//          QUSB2 PHY reset deassert. Nothing here touches the controller.
// Stage 1  read-only core identification (GSNPSID, GHWPARAMS0/1/3, DSTS).
// Stage 2  (gated by nabu-usb-init=1) core soft reset, device mode, RUN_STOP.
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

#define super IOService
OSDefineMetaClassAndStructors(NativeDwc3, IOService)

namespace {

OSSharedPtr<IOMemoryMap> g_keep;
OSSharedPtr<IOMemoryMap> g_gccKeep;
volatile uint32_t *g_regs = nullptr;
volatile uint32_t *g_gcc = nullptr;

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
  nabu_mark_num("dwc3:gdscr=", (unsigned long)gdscr, 2100);
  gwr(GCC_USB30_PRIM_GDSCR, gdscr & ~GCC_GDSCR_SW_COLLAPSE);
  if (!waitMask(g_gcc + GCC_USB30_PRIM_GDSCR / 4, GCC_GDSCR_PWR_ON, GCC_GDSCR_PWR_ON, 100000)) {
    return 0xfe;
  }

  // 19.2 MHz reference to the PHY PLL.
  gwr(GCC_USB3_PRIM_CLKREF_CLK, grd(GCC_USB3_PRIM_CLKREF_CLK) | GCC_BRANCH_ENABLE_BIT);

  // Master source: 200 MHz = GPLL0_OUT_MAIN / 3. Only configure it when the
  // register reads back something that is not already that setting -- boot may
  // have left it valid and rewriting a live RCG can glitch the branch.
  const uint32_t cfg = grd(GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CFG);
  nabu_mark_num("dwc3:rcgcfg=", (unsigned long)cfg, 2180);
  const uint32_t want = (GCC_USB_MASTER_SRC_GPLL0_MAIN << 8) | (GCC_USB_MASTER_DIV_200MHZ << 16);
  if ((cfg & 0x001f0f00u) != want) {
    gwr(GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CFG, want);
    gwr(GCC_USB30_PRIM_MASTER_CLK_SRC + GCC_RCG2_CMD,
        GCC_USB_MASTER_SRC_GPLL0_MAIN | GCC_RCG2_CMD_UPDATE);
    for (unsigned i = 0; i < 1000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  }

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
  unsigned failed = 0;
  for (unsigned i = 0; i < sizeof(branches) / sizeof(branches[0]); ++i) {
    gwr(branches[i], grd(branches[i]) | GCC_BRANCH_ENABLE_BIT);
  }
  for (unsigned i = 0; i < sizeof(branches) / sizeof(branches[0]); ++i) {
    if (!waitMask(g_gcc + branches[i] / 4, GCC_BRANCH_CLK_OFF, 0, 100000)) {
      ++failed;
    }
  }

  // Release the QUSB2 PHY from reset. The PHY itself is programmed in stage 1b;
  // the controller cannot even read GSNPSID while this is asserted in some builds.
  gwr(GCC_QUSB2PHY_PRIM_BCR, 0x1);
  (void)waitMask(g_gcc + GCC_QUSB2PHY_PRIM_BCR / 4, 0x1, 0, 100000);
  return failed;
}

} // namespace

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

  int enable = 0, init = 0;
  PE_parse_boot_argn("nabu-usb", &enable, sizeof(enable));
  PE_parse_boot_argn("nabu-usb-init", &init, sizeof(init));
  if (!enable) {
    // Driver present, hardware untouched. Flip nabu-usb=1 to run the stages.
    nabu_mark_text("dwc3:off", 2100);
    return true;
  }

  const unsigned failed = gccUsbPowerOn();
  nabu_mark_num("dwc3:clkfail=", (unsigned long)failed, 2180);
  if (failed == 0xff) {
    nabu_mark_text("dwc3:gcc-map-bad", 2100);
    return false;
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
    nabu_mark_text("dwc3:map-failed", 2100);
    return false;
  }

  const uint32_t sid = rd(DWC3_GSNPSID);
  nabu_mark_num("dwc3:sid=", (unsigned long)sid, 2260);
  const uint32_t family = sid & DWC3_GSNPSID_MASK;
  if (family != DWC3_GSNPSID_DWC_USB3 && family != DWC3_GSNPSID_DWC_USB31) {
    nabu_mark_text("dwc3:id-bad", 2100);
    return false;
  }
  nabu_mark_num("dwc3:params=", (unsigned long)rd(DWC3_GHWPARAMS0), 2340);

  // ---- stage 2: controller init (only when asked) --------------------------
  if (!init) {
    nabu_mark_text("dwc3:stage1-done", 2100);
    return true;
  }

  // dwc3_core_init(): pulse GCTL.CORESOFTRESET, wait for it to clear.
  const uint32_t gctl0 = rd(DWC3_GCTL);
  wr(DWC3_GCTL, gctl0 | DWC3_GCTL_CORESOFTRESET);
  if (!waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, DWC3_GCTL_CORESOFTRESET, 10000)) {
    nabu_mark_text("dwc3:reset-bad", 2100);
  }
  wr(DWC3_GCTL, gctl0 & ~DWC3_GCTL_CORESOFTRESET);
  if (!waitMask(g_regs + DWC3_GCTL / 4, DWC3_GCTL_CORESOFTRESET, 0, 10000)) {
    nabu_mark_text("dwc3:reset-stuck", 2100);
    return false;
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
  nabu_mark_num("dwc3:dsts=", (unsigned long)rd(DWC3_DSTS), 2340);
  nabu_mark_text("dwc3:stage2-done", 2100);

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

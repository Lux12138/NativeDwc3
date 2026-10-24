// NativeDwc3 -- stage 1: prove the DWC3 block is reachable and clocked.
//
// Sequence, following Android dwc3/core.c dwc3_core_init():
//   1. map core_base (0x0A600000, 1 MiB)   via IODeviceMemory + map(kIOMapInhibitCache)
//   2. read GSNPSID, expect 0x5533xxxx / 0x5534xxxx
//   3. soft-reset the core via GCTL.CORESOFTRESET
//   4. switch to device mode via GCTL.PRTCAPDIR(DEVICE)
//   5. clear GUSB2PHYCFG.SUSPHY
//   6. set DCTL.RUN_STOP, then read DSTS for the link state
//
// Each step paints a marker so a hang localises itself. If GSNPSID reads 0 or
// 0xffffffff the clock gates are still closed and GccUsb has to run first.

#include "NativeDwc3.h"
#include "Dwc3Registers.h"

#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSSharedPtr.h>
#include <libkern/libkern.h>

extern "C" void nabu_mark_text(const char *s, unsigned int y);
extern "C" void nabu_mark_num (const char *label, unsigned long v, unsigned int y);

#define super IOService
OSDefineMetaClassAndStructors(NativeDwc3, IOService)

namespace {

OSSharedPtr<IOMemoryMap> g_keep;
volatile uint32_t *g_regs = nullptr;

inline uint32_t rd(unsigned long off) { return g_regs[off / 4]; }
inline void wr(unsigned long off, uint32_t v) { g_regs[off / 4] = v; }

// same pattern NativeUfs uses; proved to work on this platform
static uintptr_t mapPhys(uint64_t phys, uint64_t length, OSSharedPtr<IOMemoryMap> &keep)
{
  IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
  if (!memory) { return 0; }
  keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
  memory->release();
  return keep ? reinterpret_cast<uintptr_t>(keep->getVirtualAddress()) : 0;
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

  // ---- 1. map --------------------------------------------------------------
  const uintptr_t va = mapPhys(NABU_USB0_CORE_BASE, NABU_USB0_CORE_SIZE, g_keep);
  g_regs = reinterpret_cast<volatile uint32_t *>(va);
  nabu_mark_num("dwc3:m=", (unsigned long)va, 2100);
  if (g_regs == nullptr) {
    nabu_mark_text("dwc3:map-failed", 2100);
    return false;
  }

  // ---- 2. identify ---------------------------------------------------------
  const uint32_t sid = rd(DWC3_GSNPSID);
  nabu_mark_num("dwc3:fid=", (unsigned long)(sid >> 16), 2180);   // 0x5533 / 0x5534
  nabu_mark_num("dwc3:rev=", (unsigned long)(sid & DWC3_GSNPSREV_MASK), 2260);

  const uint32_t family = sid & DWC3_GSNPSID_MASK;
  if (family != DWC3_GSNPSID_DWC_USB3 && family != DWC3_GSNPSID_DWC_USB31) {
    // 0 or 0xffffffff => clocks still gated, or the map landed somewhere else
    nabu_mark_text("dwc3:id-bad", 2100);
    return false;
  }

  // ---- 3. core soft reset --------------------------------------------------
  const uint32_t gctl0 = rd(DWC3_GCTL);
  wr(DWC3_GCTL, gctl0 | DWC3_GCTL_CORESOFTRESET);
  for (unsigned i = 0; i < 10000; ++i) { __asm__ volatile("yield" ::: "memory"); }
  wr(DWC3_GCTL, gctl0 & ~DWC3_GCTL_CORESOFTRESET);

  // ---- 4. device mode ------------------------------------------------------
  uint32_t gctl = rd(DWC3_GCTL);
  gctl &= ~DWC3_GCTL_SCALEDOWN(3);
  gctl |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_DEVICE);
  gctl |= DWC3_GCTL_DSBLCLKGTNG;
  wr(DWC3_GCTL, gctl);

  // ---- 5. unsuspend the USB2 PHY -------------------------------------------
  const unsigned ep0phys = 0;
  uint32_t p2 = rd(DWC3_GUSB2PHYCFG(ep0phys));
  p2 &= ~DWC3_GUSB2PHYCFG_SUSPHY;
  wr(DWC3_GUSB2PHYCFG(ep0phys), p2);

  // ---- 6. run --------------------------------------------------------------
  uint32_t dctl = rd(DWC3_DCTL);
  dctl |= DWC3_DCTL_RUN_STOP;
  wr(DWC3_DCTL, dctl);

  // ---- 7. link state -------------------------------------------------------
  const uint32_t dsts = rd(DWC3_DSTS);
  nabu_mark_num("dwc3:dsts=", (unsigned long)dsts, 2340);
  nabu_mark_text("dwc3:stage1-done", 2100);

  return true;
}

void NativeDwc3::stop(IOService *provider)
{
  g_regs = nullptr;
  g_keep.reset();
  super::stop(provider);
}

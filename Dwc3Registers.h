// DWC3 register definitions for SM8150 (nabu).
//
// Addresses are offsets from the controller's core_base (0x0a600000 per
// android sm8150-usb.dtsi). Values taken from drivers/usb/dwc3/core.h so they
// match the hardware documentation rather than being invented.

#ifndef NABU_DWC3_REGS_H
#define NABU_DWC3_REGS_H

// --- block geometry -------------------------------------------------------
#define NABU_USB0_CORE_BASE      0x0A600000UL
#define NABU_USB0_CORE_SIZE      0x00100000UL

// Register windows inside the block
#define DWC3_XHCI_REGS_START     0x0000
#define DWC3_GLOBALS_REGS_START  0xC100
#define DWC3_GLOBALS_REGS_END    0xC6FF
#define DWC3_DEVICE_REGS_START   0xC700
#define DWC3_DEVICE_REGS_END     0xCBFF
#define DWC3_OTG_REGS_START      0xCC00

// --- global registers -----------------------------------------------------
#define DWC3_GSBUSCFG0           0xC100
#define DWC3_GCTL                0xC110
#define DWC3_GEVTEN              0xC114
#define DWC3_GSTS                0xC118
#define DWC3_GSNPSID             0xC120
#define DWC3_GUID                0xC128
#define DWC3_GUCTL               0xC12C
#define DWC3_GHWPARAMS0          0xC140
#define DWC3_GHWPARAMS1          0xC144
#define DWC3_GHWPARAMS2          0xC148
#define DWC3_GHWPARAMS3          0xC14C
#define DWC3_GHWPARAMS4          0xC150
#define DWC3_GHWPARAMS6          0xC158

#define DWC3_GUSB2PHYCFG(n)      (0xC200 + ((n) * 0x04))
#define DWC3_GUSB3PIPECTL(n)     (0xC2C0 + ((n) * 0x04))
#define DWC3_GTXFIFOSIZ(n)       (0xC300 + ((n) * 0x04))
#define DWC3_GRXFIFOSIZ(n)       (0xC380 + ((n) * 0x04))

#define DWC3_GEVNTADRLO(n)       (0xC400 + ((n) * 0x10))
#define DWC3_GEVNTADRHI(n)       (0xC404 + ((n) * 0x10))
#define DWC3_GEVNTSIZ(n)         (0xC408 + ((n) * 0x10))
#define DWC3_GEVNTCOUNT(n)       (0xC40C + ((n) * 0x10))

// --- device registers -----------------------------------------------------
#define DWC3_DCFG                0xC700
#define DWC3_DCTL                0xC704
#define DWC3_DEVTEN              0xC708
#define DWC3_DSTS                0xC70C
#define DWC3_DGCMDPAR            0xC710
#define DWC3_DGCMD               0xC714
#define DWC3_DALEPENA            0xC720
#define DWC3_DEPCMDPAR2(n)       (0xC800 + ((n) * 0x10))
#define DWC3_DEPCMDPAR1(n)       (0xC804 + ((n) * 0x10))
#define DWC3_DEPCMDPAR0(n)       (0xC808 + ((n) * 0x10))
#define DWC3_DEPCMD(n)           (0xC80C + ((n) * 0x10))

// --- GCTL bits ------------------------------------------------------------
#define DWC3_GCTL_CORESOFTRESET  (1u << 11)
#define DWC3_GCTL_PRTCAP(n)      (((n) & (3u << 12)) >> 12)
#define DWC3_GCTL_PRTCAPDIR(n)   ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST    1u
#define DWC3_GCTL_PRTCAP_DEVICE  2u
#define DWC3_GCTL_PRTCAP_OTG     3u
#define DWC3_GCTL_SCALEDOWN(n)   ((n) << 4)
#define DWC3_GCTL_DISSCRAMBLE    (1u << 3)
#define DWC3_GCTL_DSBLCLKGTNG    (1u << 0)

// --- GUSB2PHYCFG bits -----------------------------------------------------
#define DWC3_GUSB2PHYCFG_PHYSOFTRST  (1u << 31)
#define DWC3_GUSB2PHYCFG_SUSPHY      (1u << 6)
#define DWC3_GUSB2PHYCFG_ULPI_UTMI   (1u << 4)
#define DWC3_GUSB2PHYCFG_ENBLSLPM    (1u << 8)

// --- DCTL bits ------------------------------------------------------------
#define DWC3_DCTL_RUN_STOP       (1u << 31)
#define DWC3_DCTL_CSFTRST        (1u << 30)
#define DWC3_DCTL_L0VLDO_VOLT(n) (((n) & 0x7u) << 6)
#define DWC3_DCTL_HIRD_THRES(n)  (((n) & 0x1Fu) << 24)
#define DWC3_DCTL_IGNORE_STREAM  (1u << 18)

// --- DSTS bits ------------------------------------------------------------
#define DWC3_DSTS_CONNECTSPD(n)  ((n) & 0x7u)
#define DWC3_DSTS_SOF_FN(n)      (((n) >> 8) & 0x3FFFu)
#define DWC3_DSTS_DEVCTRLHLT     (1u << 22)

// --- GSNPSID --------------------------------------------------------------
// High 16 bits identify the core; 0x5533 is DWC_usb3 / 0x5534 is usb31.
#define DWC3_GSNPSID_MASK        0xFFFF0000u
#define DWC3_GSNPSID_DWC_USB3    0x55330000u
#define DWC3_GSNPSID_DWC_USB31   0x55340000u
#define DWC3_GSNPSREV_MASK       0x0000FFFFu

#endif // NABU_DWC3_REGS_H

NativeDwc3 -- Synopsys DWC3 USB3 DRD controller for Xiaomi Pad 5 (nabu / SM8150).

Stage 1 proves the controller block is reachable: map core_base, read GSNPSID,
soft-reset, switch to device mode, clear the USB2 PHY suspend bit, run.

Register definitions follow drivers/usb/dwc3/core.h from the Android kernel so
they can be diffed against the reference implementation. Hardware parameters come
from arch/arm64/boot/dts/qcom/sm8150-usb.dtsi:

    usb0: ssusb@a600000
        reg = <0x0a600000 0x100000>
        iommus = <&apps_smmu 0x140 0x0>
        qcom,core-clk-rate = <200000000>

Clock gates (GCC_USB30_PRIM_*) must be brought up before the block answers;
GccUsb.h in NativeUfs shows the GCC access pattern for the same clock controller.

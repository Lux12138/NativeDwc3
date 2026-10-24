# NativeDwc3 设计（SM8150 / nabu）

参考实现：Android/Linux `drivers/usb/dwc3/core.c`（控制器）、`drivers/usb/dwc3/dwc3-qcom.c`（QCOM glue：时钟、复位、PHY、SMMU）、`drivers/phy/qualcomm/phy-qcom-qusb2.c`（HS PHY）、`drivers/clk/qcom/gcc-sm8150.c`（时钟与 GDSC 偏移）。

## 目标与边界

- 目标：在 XNU 下把 USB0 做成 device 模式，能枚举，最后提供一个 bulk IN 端点把内核日志推给主机（替代"卡在 panic 屏拍照"）。
- 边界：本移植的 XNU **没有 USB 设备栈**（开源树里没有 `iokit/IOKit/usb/`），所以不能复用 IOUSBDevice/IOUSBInterface；驱动直接操作 DWC3 寄存器，自建 EP0 状态机和端点队列。
- 不碰 UFS/RPMh/TCSR；不做 host 模式；不做 Type-C 角色切换（板子固定 device）。

## 硬件事实（已核对）

| 项 | 值 | 来源 |
|---|---|---|
| DWC3 core | `0x0A600000`，窗口 `0xCD00` | sm8150 DTS `usb@a600000` |
| 全局寄存器基址 | core + `0xC100`，`GSNPSID` = +`0xC120` | dwc3/core.h |
| IRQ | GIC SPI 133 | DTS `interrupts` |
| core clock | 200 MHz（GPLL0_OUT_MAIN / 3） | `qcom,core-clk-rate` |
| SMMU stream | 0x140 | DTS `iommus = <&apps_smmu 0x140 0x0>` |
| GCC base | `0x00100000` | `clock-controller@100000` |
| usb30_prim GDSC | `0x0F004`，bit0 SW_COLLAPSE，bit31 PWR_ON | gcc-sm8150.c `usb30_prim_gdsc` |
| master clk src | RCG2 `0x0F01C`，CFG +4，CMD +0 | `gcc_usb30_prim_master_clk_src` |
| master / sleep / mock_utmi | `0x0F010` / `0x0F014` / `0x0F018` | branches |
| phy_aux / com_aux / pipe | `0x0F050` / `0x0F054` / `0x0F058` | branches |
| aggre / cfg_noc | `0x0F07C` / `0x0F078` | branches |
| clkref | `0x8C008` | `gcc_usb3_prim_clkref_clk` |
| QUSB2 PHY BCR | `0x12000` | `GCC_QUSB2PHY_PRIM_BCR` |
| USB30_PRIM BCR | `0x0F000` | `GCC_USB30_PRIM_BCR` |
| USB3 PHY BCR | `0x50000` / `0x50008` | `GCC_USB3_PHY_PRIM_BCR` / DP |
| AHB2PHY BCR | `0x6A000` | `GCC_USB_PHY_CFG_AHB2PHY_BCR` |
| QUSB2 PHY | `0x088E2000` | `hsphy@88e2000` |

分支时钟：写 enable 寄存器 bit0；`CLK_OFF` 是同一寄存器 bit31，置 0 表示在跑。
GDSC：先清 `SW_COLLAPSE`，再轮询 `PWR_ON`（bit31）为 1。

## 阶段与验收

### Stage 0 —— 电源与时钟（不碰 core）

1. map GCC 窗口。
2. `usb30_prim_gdsc` 上电并轮询 `PWR_ON`。
3. `clkref` 使能（17.2/19.2 MHz 给 PHY PLL）。
4. 读 master RCG 的 CFG；不是 `GPLL0_OUT_MAIN/3` 才重写（避免对运行中的 RCG 抖动）。
5. 依次使能 master/sleep/mock_utmi/phy_aux/phy_com_aux/phy_pipe/aggre/cfg_noc，统计未清 `CLK_OFF` 的条数。
6. 释放 `GCC_QUSB2PHY_PRIM_BCR`。

验收：屏幕 `dwc3:clkfail=0`。

### Stage 1 —— 只读识别 core

1. map core 窗口（`0x0A600000`，`0xCD00`）。
2. 读 `GSNPSID`，族必须是 `0x5533` 或 `0x5534`。
3. 读 `GHWPARAMS0`，记录 FIFO/端点能力。

验收：`dwc3:sid=5533xxxx`、`dwc3:stage1-done`。
若 `sid=0/ffffffff`：时钟或 GDSC 没到位，或 PHY 复位仍asserted。

### Stage 2 —— 控制器软复位与 device 模式（`nabu-usb-init=1`）

严格按 `dwc3_core_init()`：

1. `GCTL.CORESOFTRESET` 置 1，等它自清，再清 0，等自清。
2. `GCTL.PRTCAPDIR=DEVICE`，清 `SCALEDOWN`，置 `DSBLCLKGTNG`。
3. `GUSB2PHYCFG(0).SUSPHY=0`。
4. `DCTL.RUN_STOP=1`。
5. 读 `DSTS` 记录链路状态。

验收：`dwc3:stage2-done`，`dsts` 的 `CONNECTSPD` 不再是 `0`（未接主机时为 0 属正常，接上主机后应变为 HS/SS）。

### Stage 3 —— QUSB2 HS PHY 初始化

`phy-qcom-qusb2.c` 的 SM8150 布局（`qusb2_regs_layout` v2）：

| 名称 | 偏移 |
|---|---|
| TUNE1..TUNE5 | `0x240..0x250` |
| POWERDOWN | `0x210` |
| PLL_STATUS | `0x1A0` |
| INTR_CTRL | `0x230` |

初始化表（沿用 Android 的 `qusb2_hstx_trim`/init sequence）：`PLL_ANALOG_CONTROLS_TWO=0x03`、`CLOCK_INVERTERS=0x7C`、`CMODE=0x80`、`LOCK_DELAY=0x0A`、`DIGITAL_TIMERS_TWO=0x19`、`BIAS_CTRL_1=0x40`、`BIAS_CTRL_2=0x20`、`PWR_CTRL2=0x21`、`IMP_CTRL1=0`、`IMP_CTRL2=0x58`、`TUNE1=0x30`、`TUNE2=0x29`（eFuse 可覆盖）、`TUNE3=0xCA`、`TUNE4=0x04`、`TUNE5=0x03`、`CHG_CTRL2=0`。
最后清 `POWERDOWN` 的 PLL/偏置位，轮询 `PLL_STATUS` 的 lock 位。

风险：TUNE2 的 eFuse 值必须读对，否则 HS 眼图闭合、枚举不稳。先用默认值。

### Stage 4 —— EP0 枚举

1. 申请事件缓冲（`GEVNTADRLO/HI/GEVNTSIZ`），4 KiB 对齐，物理地址必须高于 `0xC8000000`（本板 UFS/DWC3 DMA 的已知约束）。
2. `DEVTEN` 打开 connect/reset/setup 事件。
3. 事件循环：`GEVNTCOUNT` → 取事件 → 对 SETUP 走 `dwc3_ep0_setup` 的等价状态机（GET_DESCRIPTOR / SET_ADDRESS / SET_CONFIGURATION）。
4. `DEPCMDPAR/CMD` 下发 `DEPSTARTCFG`、`DEPXFERCFG`。
5. 描述符：device（VID/PID 可自定义）、config、string；不做 DFU/ADB。

验收：主机 `lsusb` 出现设备；`dmesg` 无枚举错误。

### Stage 5 —— 日志 bulk IN 端点

- 配置 EP1 IN bulk，最大包 512（HS）。
- 内核把 ring 按 4 KiB 块入队；主机侧用 libusb 连续读，写入 `nabu-log.bin`，用 `NABULOG1` 解析（与 UFS 窗口同一格式）。
- 这是"自动读日志"的最终形态：不依赖 TWRP、不依赖 UFS 写、不依赖 misc。

## 风险与前置条件

1. **时钟未开就写 core 会复位整板**（2026-10-24 实测）。所以 Stage 0 必须先过，Stage 2 必须由 boot-arg 显式打开。
2. **SMMU**：DWC3 的 stream 0x140 需要 bypass 映射。复用 `NativeUfs::smmuBypassStream` 的 quirk（reserve last context bank，S2CR TYPE=TRANSLATE，CBNDX 指向它），只换 stream 号。
3. **DMA 地址**：可用区间从 `0xC8000000` 起；`0xF080F000` 曾触发复位。事件缓冲/TRB 用 `IOMallocContiguous` 并校验物理地址。
4. **中断**：GIC SPI 133。XNU 侧需要 `IOInterruptController`/`IOInterruptSource` 绑定，或者在 Stage 4 先用轮询（`GEVNTCOUNT` 自旋），把中断留到最后。
5. **XNU 无 USB 栈**：不能 `IOService::publish` 成 `IOUSBDevice`；先做成一个普通 IOService + 用户态不可见的字符通道。主机侧不依赖 macOS 的 USB 栈，用 libusb。
6. **PHY 校准**：TUNE2 的 eFuse 值、IMP 校准。默认值能跑通枚举，但高速稳定性要实测。
7. **角色/供电**：板子固定 device，VBUS 由主机提供；不要写 OTG 控制寄存器。

## 代码结构

```
NativeDwc3.h        IOService 子类
NativeDwc3.cpp      Stage 0/1/2（本文件）
GccUsb.h            SM8150 GCC USB 偏移（核对自 gcc-sm8150.c）
Dwc3Registers.h     DWC3 寄存器与位域
Info.plist          IOKit 匹配（provider 由 board.toml 注入）
```

Stage 3/4/5 后续分别落在 `Qusb2Phy.h`、`Dwc3Ep0.cpp`、`Dwc3LogEp.cpp`。

## 启动参数

| 参数 | 默认 | 作用 |
|---|---|---|
| `nabu-usb` | 0 | 1 = 执行 Stage 0/1 |
| `nabu-usb-init` | 0 | 1 = 追加 Stage 2（写 core） |

默认全关：驱动可以一直在内核里，板子照常启动。

## 与 Android 的差异

- Android 用 `clk_bulk`/`reset_control`/`phy` 框架；这里直接写寄存器，顺序照抄，错误处理简化为屏幕 marker。
- Android 的 `dwc3_qcom_interconnect_init`（NoC QoS）省略：本板 bootloader 已配好带宽。
- Android 用 `dma_alloc_coherent`；这里用 `IOMallocContiguous` + 手工 `IODMACommand` 或物理地址。
- Android 的 extcon/role switch 省略。

## 硬件实测（2026-10-24）

Stage 0 已跑通，读数来自内核 ring（`dwc3:*` 行）：

| 读数 | 值 | 含义 |
|---|---|---|
| `gdscr` | `0xf8222000` | PWR_ON=1，SW_COLLAPSE=0，GDSC 已上电 |
| `clkfail` | `0` | 8 路分支时钟 CLK_OFF 全部为 0 |
| `rcgcfg` | `0x105` | SRC=GPLL0_OUT_MAIN，hid_div=5，即 200 MHz（字段在 bit0..7，不是 bit16） |
| `rcgm/n/d` | `0` | MND 模式未被使用；不需要重配 RCG |
| `cap/hcsp/gsbus/guid/sid` | `0` | core 全域读 0 |

结论：GCC 侧（GDSC、clkref、分支时钟、`GCC_USB30_PRIM_BCR` 释放）已经正确，但 core 仍然完全不响应。

## core 读 0 的根因：PHY 没有供电

SM8150 的 USB HS PHY 不是 QUSB2，而是 **7nm femto PHY**：

```
usb_1_hsphy: phy@88e2000 {
    compatible = "qcom,sm8150-usb-hs-phy", "qcom,usb-snps-hs-7nm-phy";
    reg = <0 0x088e2000 0 0x400>;
    clocks = <&rpmhcc RPMH_CXO_CLK>;
    resets = <&gcc GCC_QUSB2PHY_PRIM_BCR>;
};
```

对应驱动 `drivers/phy/qualcomm/phy-qcom-snps-femto-v2.c`，它需要三路 PMIC LDO：

- `vdda-pll`（0.88 V，PLL）
- `vdda33`（3.3 V）
- `vdda18`（1.8 V）

没有这些电源，PHY 寄存器读 0（实测 `phy-status=0`），DWC3 core 也就没有 UTMI/PIPE 时钟，所有寄存器读 0。

XNU 当前没有 PMIC（SPMI/RPMh）驱动，所以 Stage 1 之前必须先补：

1. PMIC LDO 上电（或确认 XBL 已经打开、且没有被 AP 关掉）；
2. `RPMH_CXO_CLK` 到 PHY 的 ref clock；
3. 然后才轮到 femto PHY 的 init sequence 与 PLL lock。

在那之前，驱动默认 `nabu-usb=0`，不碰硬件。

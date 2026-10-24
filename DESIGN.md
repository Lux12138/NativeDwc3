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

## 2026-10-24 深夜：core 已通，PHY 仍未响应

在 RPMh 与 BCR 修正之后：

| 读数 | 值 | 含义 |
|---|---|---|
| `rpmh:l5v` | `0x80000370` | vdda-pll = 880 mV（回读确认） |
| `rpmh:l2v` | `0x80000c00` | vdda33 = 3072 mV |
| `rpmh:l12v` | `0x80000708` | vdda18 = 1800 mV |
| `rpmh:xov` | `3` | xo.lvl 已使能（PHY ref clock） |
| `dwc3:cap` | `0x01100020` | XHCI 1.1，core 已上电 |
| `dwc3:gsbus` | `0x2222000e` | GSBUSCFG0 正常 |
| `dwc3:guid` | `0x17013c42` | GUID 正常 |
| `dwc3:params0` | `0x4020400a` | GHWPARAMS0 正常 |
| `dwc3:sid` | `0x33313130` | **DWC_usb31（DWC31_IP = 0x3331）**，不是 0x5533 |
| `dwc3:dsts` | `0x20004` | device mode 已运行 |
| `phy:r0/r4/0x1a0/0x94` | `0` | PHY 数字接口无响应 |
| `phy:common2(0x5c)` | `0xc1c1c1c1` | 总线返回的垃圾模式 |

两个决定性修正：

1. **BCR 必须显式写 0 释放**。qcom reset 没有自清：写 1 只 assert，读回一直是 1，PHY 和 core
   因此全程处于复位。改成 assert → 延时 → 0 之后，core 立刻返回 XHCI 头和全部全局寄存器。
2. **GSNPSID 家族要包含 0x3331**。Linux `dwc3/core.h` 的 `DWC31_IP = 0x3331`，
   本板读出 `0x33313130`，此前按 0x5533/0x5534 判断所以误判为“无 ID”。

仍然阻塞的点：PHY 数字接口读 0。三路模拟电源与 XO 已回读确认到位，所以下一步是
PHY 的 AHB 访问条件（`usb_phy_cfg_ahb2phy` 的时钟/复位），而不是供电。

## 2026-10-24 深夜：EP0 基础设施完成，PHY 仍是墙

已完成并回读验证：

| 项 | 值 |
|---|---|
| RPMh LDO | `l5=880mV`、`l2=3072mV`、`l12=1800mV`，`xo.lvl=3` |
| DWC3 core | `cap=0x01100020`（XHCI 1.1）、`sid=0x33313130`（DWC_usb31） |
| Device 模式 | `gctl2=0x2001`（PRTCAPDIR=2） |
| 速度 | `dsts=0xd20001`（HS/FS 已请求） |
| SMMU | 独立条目 `slot=6`、`bank=36`、`SMR=0x80000140`、`S2CR=0x24`（UFS 的 bank 未动） |
| 事件环 | 4 KiB 物理连续 @ `0xcaa4xxxx`，GEVTEN/DEVTEN 已配置 |
| 事件 | 0 |
| PC 端枚举 | 无新设备 |

### PHY 的 AHB 端口在 XNU 下没有响应

对 `0x088E2000`（DTS 的 `hsphy@88e2000`）无论读哪些偏移、无论是否复位、无论是否执行 femto
序列，读回的都是总线默认 pattern：

- `0x00` → `0`
- `0x5c`（COMMON2）→ `0xc1c1c1c1`
- `0x54`（COMMON0）→ `0x88888888`

写入后再读不变，说明**写也没有落到 PHY**。扫 64 KiB 窗口（`0x88e0000`）会在未解码的页上挂死，
所以只能按单页访问。

排除项：

1. 模拟供电：RPMh 回读确认。
2. 复位：`GCC_QUSB2PHY_PRIM_BCR` 已 assert→deassert，读回 0。
3. AHB 桥复位：`GCC_USB_PHY_CFG_AHB2PHY_BCR` 已释放，`0x6A004` 读回 1。
4. XBL 遗留配置：不复位直接读，仍是同样 pattern，说明 XBL 跳转前已经关掉 PHY。
5. Mu UEFI：`nabuPkg` 没有 USB 驱动，`UsbfnDwc3Dxe` 只处理端点；fastboot 的 USB 由 XBL 提供。

### Android 内核的对照（TWRP dmesg）

```
androidboot.usbcontroller=a600000.dwc3
msm-dwc3 a600000.ssusb: USB core frequency = 200000000
dwc3 a600000.dwc3: changing max_speed on rev 00000000
msm-dwc3 a600000.ssusb: DWC3 exited from low power mode
```

关键点：**Android 内核读到的 GSNPSID 也是 0（rev 00000000）**，它并不依赖这个值，而是由 QCOM
downstream 的 PHY 驱动完成 PHY 上电，然后在 `msm_dwc3_resume` 里退出低功耗模式。mainline 的
femto 驱动与它并不等价。

### 下一步

PHY 这一层需要换证据来源，不能再靠盲写寄存器：

1. 从 TWRP 的 `/proc/kcore` 按 ELF 的 `p_paddr` 取 `0x88E2000`，得到 Android 下 PHY 寄存器的
   真实值，和 XNU 的 pattern 对比。
2. 或者移植 QCOM downstream 的 SM8150 HS PHY 驱动（`phy-msm-snps-hs` 系列），它才是这块板
   在 Android 上实际使用的代码。
3. 在拿到对照值之前，`nabu-usb` 保持关闭，驱动不碰 USB 硬件。

## 2026-10-24 23:28：PHY 从未“死”，阻塞改在 CURMOD

Lineage 的 SM8150 设备树给出了两个此前不知道的事实：

```
hsphy@88e2000 {
    compatible = "qcom,usb-hsphy-snps-femto";
    reg = <0x88e2000 0x110>,      // 窗口只有 0x110
          <0x007801f8 0x4>;       // eFuse RCAL
    vdd-supply     = <&pm8150_l5>;   // 880 mV
    vdda18-supply  = <&pm8150_l12>;  // 1.8 V
    vdda33-supply  = <&pm8150_l2>;   // 3.072 V
};
```

由此修正了两处：

1. 之前按 qusb2 偏移读的 `0x1a0`、`0x240` 全部**在 0x110 窗口之外**，读到的是别的模块。
2. mainline 的 femto 驱动缺少 QCOM 的 `RTUNE_SEL`（0xB4）和 RCAL；本板 eFuse RCAL 为
   `0x01e00000`（非 0），所以本来就不需要 RTUNE_SEL。

**PHY 的写测试（决定性）**：

```
COMMON0 读 0x88888888 → 写 0x88888808 → 读回 0x08080808
```

值随写变化，只读位被屏蔽。**AHB 写和读都到达 PHY，PHY 一直是活的。**

## 现在的真正阻塞：core 没有切到 device

| 寄存器 | 值 | 含义 |
|---|---|---|
| `GCTL` | `0x2001` | PRTCAPDIR=2（DEVICE）已写入 |
| `GSTS` | `0x7e800000` | CURMOD = 0（仍为 OTG/未决定） |
| `DSTS` | `0xd20001` | 按 device 寄存器读出，CONNECTSPD=1 |

下一步只需解决“写了 PRTCAPDIR 但 CURMOD 不变”这一件事：查 Synopsys databook 里
`GCTL.PrtCapDir` 的写入条件（是否要求先做一次 `GCTL.CoreSoftReset` 且保持
`GUSB2PHYCFG.PHYSOFTRST` 时序），或者确认该 core 的 PRTCAPDIR 由外部 strap 固定。

PHY、供电、SMMU、事件环、EP0 缓冲这五块基础设施现在都已验证或就绪。

## 2026-10-24 23:36：最后一道门是 CURMOD

这一轮把 DWC3 device 初始化里缺的步骤补齐了：

| 补齐的步骤 | 来源 |
|---|---|
| `GCTL` 整体写 device 模式（不能保留旧位） | dwc3/core.c `dwc3_set_prtcap` |
| `GUSB2PHYCFG.PHYSOFTRST` 脉冲 | dwc3/core.c `dwc3_core_init` |
| `DCTL.CSFTRST` 脉冲 | dwc3/core.c `dwc3_gadget_init` |
| PHY `SUSPEND_N` 清零（resume） | `msm_hsphy_set_suspend(0)` |
| PHY `SLEEPM=0`、`OPMODE=NORMAL` | `phy_power_on` |
| RCAL / RTUNE_SEL | `phy-msm-snps-hs.c` |

结果：

```
GCTL  = 0x2001    PRTCAPDIR = 2 (DEVICE)
DCTL  = 0x00f00000 RUN_STOP = 0
GSTS  = 0x7e800000 CURMOD = 0   ← 仍然不是 device
DSTS  = 0xd20001   CONNECTSPD = 1
```

**PHY、供电、SMMU、事件环、EP0 缓冲全部就绪并已验证，唯一剩下的就是 CURMOD 不切换。**

QCOM 的 `dwc3-msm.c` 不是在 probe 里写 PRTCAPDIR 就完事，它用 `id_state`
（ID 引脚 / extcon）驱动 `msm_dwc3_set_mode()`，由 VBUS/ID 事件决定 host 或 device。
下一步应当移植这条路径，或者在写 PRTCAPDIR 之前先把 ID 状态置为 ground：

```c
case DWC3_ID_GROUND:  dwc3_set_prtcap(dwc, DWC3_GCTL_PRTCAP_DEVICE); break;
case DWC3_ID_FLOAT:   dwc3_set_prtcap(dwc, DWC3_GCTL_PRTCAP_HOST);   break;
```

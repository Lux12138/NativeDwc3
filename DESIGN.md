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

## 2026-10-24 23:50：最后一道门是 VBUS 检测

把 `DWC3_DCTL` 的软复位、`GUSB2PHYCFG.PHYSOFTRST`、PHY 的完整 `power_on`（DATAPATH override、
TERMSEL、XCVRSEL）都补齐之后：

```
OSTS  = 0        VBUSVLD = 0   ← core 认为没有 VBUS
DCTL  = 0x00f00000            KEEP_CONNECT(bit19) 写不进去（只读）
GUSB2PHYCFG |= ULPIEXTVBUSDRV(bit17)   写了，OSTS 仍 0（UTMI 模式忽略该位）
GSTS.CURMOD = 0               因为不认为有 VBUS，也就没有进入 device
```

DWC3 在 device 模式下必须看到 VBUS 才 connect（D+ 上拉）。这块板的 VBUS 检测链路是：

```
USB VBUS -> PM8150 充电器 (SPMI) -> extcon -> dwc3-msm 的 msm_dwc3_vbus()
                                             -> 清 DCTL.RUN_STOP -> connect
```

XNU 没有 SPMI/充电器驱动，所以这条链断了。软件侧的替代都被硬件拒绝：

- `DCTL.KEEP_CONNECT`：只读
- `GUSB2PHYCFG.ULPIEXTVBUSDRV`：只对 ULPI 接口有效，本板是 UTMI
- `GSTS/OSTS.VBUSVLD`：只读状态位

### 下一步

要让 XNU 的 USB device 真正连上，需要补下面任意一条：

1. **SPMI + PM8150 充电器的 VBUS 检测**（最小实现：读 VBUS 状态位，然后清 `DCTL.RUN_STOP`）。
2. **gpio 强制 VBUS 有效**：若 PMIC 的 VBUS 输入可以配置成常有效，由 SPMI 写一次即可。
3. 若 Mu UEFI 在运行时能把 VBUS 状态保留给 XNU，可省掉 SPMI（需要确认 XBL/UEFI 是否保持）。

到这为止，DWC3 侧（PHY、core、SMMU、事件环、EP0 缓冲、device 初始化）全部就绪且逐项验证，
唯一缺口是“core 看不到 VBUS”。

## 2026-10-25 00:00：VBUS 走 PMIC，SPMI 已通电但映射未通

### 已验证

| 项 | 结果 |
|---|---|
| PHY 内部 VBUS 检测（VBUSVLDEXT0/SEL0 = 0） | `OSTS` 仍 0；SM8150 的 femto PHY 没有 VBUS 引脚 |
| ULPIEXTVBUSDRV（bit17） | 已回滚；UTMI 模式下无效 |
| PMIC arbiter 版本 | `0x50000000`（V5），总线与电源可用 |
| mapping table 查询 ppid=0x210 | `valid=0`，未命中 |

### 结论

DWC3 的 `OSTS.VBUSVLD` 只能由 PMIC 的 VBUS 检测链路提供：

```
USB 5 V -> PM8150 charger (SPMI) -> extcon -> dwc3-msm -> 清 RUN_STOP / PHY VBUS 有效
```

XNU 目前缺的就是这一段的 SPMI 读/配置。arbiter 已经在 V5 模式工作，剩下的是
`pmic_arb_parse_mapping_table` 的位序对照，以及 PM8150 的 SID 与 charger
`USBIN_INT_RT_STS` 的准确地址。

### 下一步

1. 对照 `drivers/spmi/spmi-pmic-arb.c` 的 `pmic_arb_parse_mapping_table()`，
   修正 16 级树的位序（当前实现：IS_1 取 bit0/bit8，IS_0 取 bit9/bit17）。
2. 确认 AP 的 EE（`qcom,ee`）与 PM8150 的 SID（通常 2），再读 `0x1010`。
3. 若 VBUS=1 但 PHY 仍看不到，检查 PMIC 的 VBUS 输出是否被 XBL 关掉（可能需要写
   charger 的 `USBIN_CMD_IL`/`OTG_CMD` 使能检测）。

---

# 2026-10-25 02:00 — vendor 源码到手后的实测记录（run1..run6）

## 决定性事实：绑定的驱动不是 qusb2

TWRP 里 `ls -l /sys/bus/platform/devices/88e2000.hsphy/driver` 给出
`msm-usb-hsphy`，即 `drivers/usb/phy/phy-msm-snps-hs.c`。在此之前我们按
`phy-qcom-qusb2.c` 和另一套 femto 语义写寄存器，方向全错。

源码：MiCode/Xiaomi_Kernel_OpenSource，分支 `nabu-r-oss`。

## 真实寄存器表（phy-msm-snps-hs.c）

    0x3c UTMI_CTRL0   OPMODE_MASK 0x18 / OPMODE_NONDRIVING 0x08 / SLEEPM bit0 / TERMSEL bit5
    0x40 UTMI_CTRL1   XCVRSEL bit0
    0x50 UTMI_CTRL5   POR bit1
    0x54 COMMON0      RETENABLEN bit3 / FSEL_MASK 0x70
    0x58 COMMON1      VBUSVLDEXTSEL0 bit4 / PLLBTUNE bit5
    0x5c COMMON2      VREGBYPASS bit0
    0x60 CTRL1        VBUSVLDEXT0 bit0
    0x64 CTRL2        AUTO_RESUME bit0 / SUSPEND_N bit2 / SUSPEND_N_SEL bit3
    0x94 CFG0         DATAPATH_OVERRIDE bit0 / CMN_CTRL_OVERRIDE bit1
    0xa0 REFCLK_CTRL  REFCLK_SEL_DEFAULT = 0x2
    0xb4 RTUNE_SEL    bit0
    0x6c..0x78        PARAMETER_OVERRIDE_X0..X3

注意 0x38/0xb4 在 qusb2 里分别是 PLL_STATUS 和 PORT_POWERDOWN，在本 PHY 上
不是。0xb4 写 bit0 是打开外部电阻调谐，不是“上电”。

## qcom,param-override-seq 的真实方向

`hsusb_phy_write_seq()` 的实现是 `writel(seq[i], base + seq[i+1])`，即
值写进第二个数指向的偏移。板子自己的 DT（从 TWRP 的
/sys/firmware/devicetree/base 读出）是 `<4 0x70 0x2c 0x74>`，因此：
`0x70 <- 0x04`、`0x74 <- 0x2c`。源码 DTS 里的默认值是 `<0x43 0x70>`。

## 厂商 msm_hsphy_init() 顺序（照抄，顺序有意义）

    CFG0 |= CMN_CTRL_OVERRIDE
    CTRL5 |= POR
    COMMON0 FSEL = 0
    COMMON1 |= PLLBTUNE
    REFCLK_CTRL = 0x2
    COMMON1 |= VBUSVLDEXTSEL0
    CTRL1   |= VBUSVLDEXT0
    param_override_seq
    rcal: (!rcal_code) -> 0xb4 RTUNE_SEL
    COMMON2 |= VREGBYPASS
    CTRL2   |= SUSPEND_N_SEL | SUSPEND_N
    UTMI_CTRL0 |= SLEEPM
    CTRL5 POR = 0
    CTRL2 SUSPEND_N_SEL = 0     （SUSPEND_N 保持 1）
    CFG0 &= ~CMN_CTRL_OVERRIDE

PHY 参考时钟在 DTS 里是 `clocks = <&clock_rpmh RPMH_CXO_CLK>`，
不是 GCC 的 usb3 clkref 分支；两个 clkref 分支开着也无害。

## DWC3 语义修正

- `DCTL.RUN_STOP` = 1 才是 run（Linux dwc3_gadget_run_stop 在 is_on 时置位）。
  上一版先置 1 再清 0，核心一直 halted：`DSTS.DEVCTRLHLT(bit22)=1` 实测证实。
- `DCFG.DEVSPD`：core.h 为 `HIGHSPEED (0 << 0)`、`SUPERSPEED (4 << 0)`。
  清 0 就是 HS，不要“修正”成 1（那是 FullSpeed）。
- `DSTS.USBLNKST = (v >> 18) & 0xf`，值 4 = SS.Disabled，是 HS 设备“未连接”的常态。

## QSCRATCH（wrapper，偏移 0xF8800）

    +0x08 GENERAL_CFG   PIPE_UTMI_CLK_SEL bit0 / PIPE3_PHYSTATUS_SW bit3 / PIPE_UTMI_CLK_DIS bit8
    +0x10 HS_PHY_CTRL   UTMI_OTG_VBUS_VALID bit20 / SW_SESSVLD_SEL bit28
    +0x28 CGCTL         置 0x18 打开 RAM 主时钟

`dwc3_override_vbus_status()` 就是把 bit20 写成 VBUS 状态。本板上写这个位会
把 AHB 从设备锁死：核心随后任何寄存器访问都不返回（run5 卡在 dctl-pre 之后、
run6 卡在 qs:hs 之前，两次都需要长按电源键）。`PIPE_UTMI_CLK_DIS` 的“先关再开”
同样会锁死（run3）。

## 实测记录

| run | 变化 | 结果 |
|-----|------|------|
| 1 | 旧版（RUN_STOP 先 1 后 0） | halt，evtcnt=0 |
| 2 | RUN_STOP 保持 1 | ran=1，dsts=920001，osts=0，evtcnt=0 |
| 3 | + QSCRATCH UTMI 时钟时序（含 CLK_DIS） | 锁死 |
| 4 | + 厂商 PHY 序列 + VBUS 覆盖 + CLK_SEL/PHYSTATUS_SW | 锁死（qs:hs 之后） |
| 5 | 去掉 CLK_SEL，只留 VBUS 覆盖 | 锁死在 DCTL 写（无 dctl-post） |
| 6 | 先 RUN 再 VBUS 覆盖 | DCTL 成功、ran=1，随后写 VBUS 位即锁死 |

run6 全程主机侧 `pnputil` 监视：只有 fastboot(18d1:d00d) 和 TWRP adb(18d1:d001)，
没有任何第三个 USB 设备出现，说明 D+ 上拉从未拉起。

## 电源已排除

TWRP 正常态（Android 内核）：

    pm8150_l5  enabled users=6  880000 uV   <- vdd
    pm8150_l12 enabled users=2 1800000 uV   <- vdda18
    pm8150_l2  enabled users=3 3072000 uV   <- vdda33
    hsphy@88e2000 enabled users=1           <- PHY 自己提供的 dpdm regulator

三者与我们 RPMh 投的 880/1800/3072 完全一致。`dpdm` regulator 的作用是把 PHY
置于 non-driving（让 PMIC 做充电检测），不是我们必须补的东西。

## 当前假设

核心在 `RUN_STOP=1` 且 VBUS 有效时会立刻驱动 UTMI/PHY 进入连接流程；此时若 PHY
没有真正工作在（没有 60 MHz UTMI 时钟），两侧互锁，表现为 AHB 挂死。因此下一步
不再从 core 侧强推 VBUS，而是让 PHY 用自己的 VBUS 比较器
（`VBUSVLDEXTSEL0=0`/`VBUSVLDEXT0=0`）自行判断，观察 `OSTS.VBUSVLD` 是否变 1。

---

# 2026-10-25 02:45 — run10..run14：上拉、VBUS 与 SPMI 的边界

## run9（重要）：QSCRATCH VBUS 覆盖写完，之后一启动就死锁

    qs:hs=10100000     ← 位确实写进去了
    qs:osts=0          ← 但 OTG 的 VBUSVLD 仍为 0
    dwc3:dctl-pre=90f00000 → 无 dctl-post（写 DCTL 时 AHB 锁死）

即使先把 `UTMI_OTG_VBUS_VALID` 写进去、`OSTS` 仍显示 0，随后 RUN 依然死锁。
结论：**QSCRATCH 的 HS_PHY_CTRL bit20 在本板上不可用**，这条软件 VBUS 路线封死。

## run10：直接写 PHY 的 TERMSEL / XCVRSEL，电气上无反应

厂商 `drive_dp_pulse()` 直接改 `UTMI_CTRL0.TERMSEL`、`UTMI_CTRL1.XCVRSEL`、
`CFG0` 的两个 override，就能让 PHY 自己发 D+ 脉冲。照抄并保持打开：

    phy:term=21212121   TERMSEL=1、SLEEPM=1 写入并读回确认
    phy:xcvr=01010101   XCVRSEL=1
    phy:cfg0t=03030303  CMN + DATAPATH override 均置位
    → 主机侧 pnputil 监视：没有任何新 USB 设备出现在 D+ 上

写入有效、读回保持，说明 override 没有被核心覆盖；但电气上毫无变化。
这指向 PHY 模拟侧（上拉电路/模拟电源）没有工作，而不是寄存器配置错误。

## run12：bootloader 移交时的 USB 状态（nabu-usb-keep=1 只读探针）

    keep:gctl=102001    PRTCAPDIR=HOST
    keep:dctl=f00000    RUN_STOP=0
    keep:dsts=d3953c    CONNECTSPD=4(SS)、DEVCTRLHLT=1
    keep:dcfg=80805     DEVSPD=5 (SuperSpeedPlus)
    keep:g2cfg=102500   、keep:qsg=4、keep:qsh=10000000

ABL 在移交前把控制器切回 host + SS + halt，没有留下可继承的设备态，
因此“沿用 bootloader 已经点亮 PHY”的思路不成立。

## run11/13/14：SPMI 能映射了，但 mapping table 格式未解

`mapPhys(0x0C440000, 0x1100)` 一直失败（0x1100 不按页对齐），改成 0x2000 后：

    spmi:ver=50000000     PMIC arbiter V5 在线

0xB00 处的表**非空**，前 8 项原始值：

    0x01040400 0x01040500 0x01040600 0x01040900
    0x01042c00 0x01043600 0x01043700 0x01043900

无论按 MSB-first 还是 LSB-first 的“每层一项、bit8/bit17 为 valid”解析，
`valid` 都是 0。说明这张表的编码不是 QCOM 公开的那种结构，需要另找格式
（可能是每层多个 u32，或以 0x0104/0x0105 开头的高字节有其他含义）。
在读通它之前，PMIC 的 LDO 实际状态读不到。

## 当前唯一待验证的假设

PHY 数字寄存器完全在线（写入/读回一致），但所有能让 D+ 出现上拉的手段都
没有电气效果。剩下的解释只有 **PHY 的模拟电源（vdda18/vdda33）在 XNU 下
没有真正到位**。RPMh 的 `l5v/l12v/l2v` 回读只是 RSC 的命令回执，不等于
PMIC 输出；要证实只能通过 SPMI 读 PM8150 的 LDO 状态。

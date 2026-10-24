# DWC3 的下一个前置：RPMh / PMIC 供电

## 实测证据（2026-10-24）

| 观察 | 结果 |
|---|---|
| XNU 下的 GCC 寄存器 | 可读，`gdscr=0xf8222000`、8 路分支 `CLK_OFF=0` |
| XNU 下的 DWC3 core（0x0A600000） | 全部偏移读 0（`cap/hcsp/gsbus/guid/sid`） |
| XNU 下的 HS PHY（0x088E2000） | 全部读 0，`PLL_STATUS=0` |
| TWRP 下的 USB | adb over USB 正常工作 → PHY/DWC3 硬件本身没问题 |
| TWRP 的 regulator 列表 | `pm8150_l5`（0.88 V）enabled；`pm8150_l7`（1.8 V）、`pm8150l_l7`（2.856 V）disabled |

结论：XNU 启动后 PMIC 没有为 7nm HS PHY 保持供电（`vdda-pll` 可能由 XBL 开过，
`vdda18`/`vdda33` 处于关闭），core 因此没有 UTMI/PIPE 时钟，所有寄存器读 0。
GCC 侧的上电与时序已经正确，缺的只是 PMIC 投票。

## 硬件

SM8150 的 RPMh：

```
apps_rsc: rsc@18200000 {
    compatible = "qcom,rpmh-rsc";
    reg = <0x18200000 0x10000>, <0x18210000 0x10000>, <0x18220000 0x10000>;
    qcom,tcs-offset = <0xd00>;
    qcom,drv-id = <2>;
    qcom,tcs-config = <ACTIVE_TCS 2>, <SLEEP_TCS 3>, <WAKE_TCS 3>, <CONTROL_TCS 1>;
};
```

要发一条 regulator 请求，最小链路是：

1. **SMEM**：找到 `smem` 分区（GPT 里没有独立分区，地址来自 XBL 的 SMEM 表）。
2. **cmd_db**：从 SMEM 的 `cmd_db` 条目读资源表，把 `pm8150_l7` 这样的名字映射成
   RPMh 的 `(resource id, address)`。
3. **TCS**：向 `apps_rsc` 的 drv-2 的 ACTIVE TCS 写命令（`RPMH_CMD_REGULATOR` 类），
   设置电压与 enable，然后写 `CMD_SOLVER` 触发。
4. 轮询 TCS 完成位。

## 为什么这不是 DWC3 驱动内部能解决的事

- 命令格式、资源 id 表都在 SMEM/cmd_db，不在 DTS 的固定寄存器里。
- 需要先有一个最小的 SMEM 读取器（GPT 无 smem 分区，地址由 XBL 传入或从
  `/soc/smem` 的保留内存扫描）。
- 写错 TCS 或资源 id 会影响其他子系统的供电，风险高于目前所有 UFS 改动。

## 两个可行路线

### A. 最小 RPMh regulator 投票（推荐）

只实现：

- SMEM 定位 + cmd_db 解析（只读）。
- ACTIVE TCS 单命令：给 USB PHY 的三路 LDO 发 enable + 电压。
- 完成后立即回到 DWC3 Stage 1。

工作量：SMEM/cmd_db 解析约 200 行，TCS 命令约 150 行，验证以“PHY PLL lock 位”为准。

### B. 让 XBL 保持供电

在 `fastboot boot` 之前由 XBL/UEFI 打开 LDO 并**不清除投票**。这需要在 Mu UEFI 或
XBL 侧改动，超出本仓库范围；而且 AP 侧 RPMh 仍可能在睡眠态把资源降档。

## 验收标准

1. XNU 下 `0x088E2000` 的 `PLL_STATUS` bit5 = 1。
2. `0x0A6C120` 读回 `0x5533xxxx` / `0x5534xxxx`。
3. `dwc3:stage1-done` 出现在日志窗口（主机可读）。

在此之前，`nabu-usb` 默认 0，驱动不碰 USB 硬件。

# Icebox 数控电源 — 固件开发参考（circuit.md）

> 本文件面向**固件开发**，汇总 MCU 需要交互的全部硬件信息：GPIO 分配、NCP81239 I²C 寄存器、换算公式、初始化序列、已知坑。
> 数据来源：2026-07-25 用 yuanlitu MCP 从原理图逐网络提取的实测网表 + NCP81239-D 数据手册。硬件细节/设计依据见 `note.md`，本文件只保留固件视角。
> **单位约定**：电阻分压比 `DIV = 7.847`（Rtop=68.47k / Rbot=10k）；RSENSE1=RSENSE2=2mΩ；R_CS1=R_CS2=8.2k。

---

## 0. 系统总览

- **主控**：ESP32-S3-WROOM-1-**N16R8**（16MB Flash + 8MB Octal PSRAM）
  - ⚠️ R8 版工作温度上限 **65°C**（不是 85°C），功率区发热需注意
  - ⚠️ IO35/36/37 被内部 PSRAM 占用，固件**不可使用**
  - WiFi 开启时 **ADC2 不可用**，所有模拟量必须用 **ADC1**
- **电源控制器**：NCP81239（4 开关 Buck-Boost），通过 **I²C** 设定输出电压/限流/读回监测
- **交互**：五向拨轮开关(U6) + TFT(ST7789V2, SPI) + 双风扇(PWM+TACH) + 3 路外置 NTC
- **供电**：12V → 78M05 → 5V(NCP VCC/VDRV) → AMS1117 → 3.3V(MCU)

---

## 1. ESP32-S3 GPIO 分配表（实测网表）

| 脚 | GPIO | 网络名 | 功能 | 固件配置 |
|:--:|:----:|:------|:-----|:---------|
| 3 | EN | EN | 芯片复位 | 硬件 RC + 按键，非 GPIO |
| 4 | IO4 | — | **空闲**（预留 ADC1_CH3） | — |
| 5 | IO5 | NTC4_ADC | NTC 座 NTC_N（上拉 3.3k） | ADC1_CH4 输入 |
| 6 | IO6 | NTC2_ADC | NTC 座 NTC_L1（上拉 33k） | ADC1_CH5 输入 |
| 7 | IO7 | NTC3_ADC | NTC 座 NTC_L2（上拉 33k） | ADC1_CH6 输入 |
| 8 | IO15 | KEY1 | 拨轮 U6.1 | 数字输入，内部/外部上拉，active-low |
| 9 | IO16 | KEY3 | 拨轮 U6.2 | 同上 |
| 10 | IO17 | SDA | I²C 数据（4.7k↑3V3） | I²C master SDA |
| 11 | IO18 | SCL | I²C 时钟（4.7k↑3V3） | I²C master SCL |
| 13 | IO19 | D− | USB D−（原生 USB） | USB-Serial-JTAG |
| 14 | IO20 | D+ | USB D+ | USB-Serial-JTAG |
| 18 | IO10 | D_RST | TFT RST | GPIO 输出，硬复位屏 |
| 19 | IO11 | D_MOSI | TFT MOSI | FSPID (SPI2) |
| 20 | IO12 | D_CLK | TFT CLK | FSPICLK (SPI2) |
| 21 | IO13 | D_CS | TFT CS | FSPICS0 |
| 22 | IO14 | D_DC | TFT DC（数据/命令选择） | GPIO 输出 |
| 23 | IO21 | P_EN | **NCP81239 EN**（使能功率级） | GPIO 输出，上电默认低=禁用 |
| 24 | IO47 | D_BLK | TFT 背光 PWM | LEDC **正相**（高=亮），~1kHz |
| 25 | IO48 | KEY2 | 拨轮 U6.T（按下） | 数字输入，active-low |
| 27 | IO0 | BOOT | 启动模式 | 硬件按键，非 GPIO |
| 31 | IO38 | FAN1_PWM | 风扇1 PWM | LEDC **反相**（见坑#3），25kHz 开漏 |
| 32 | IO39 | FAN2_PWM | 风扇2 PWM | 同上 |
| 33 | IO40 | FAN1_TACH | 风扇1 转速 | PCNT 计数输入（10k↑3V3） |
| 34 | IO41 | FAN2_TACH | 风扇2 转速 | 同上 |
| 35 | IO42 | INT | NCP81239 INT 中断 | GPIO 输入，下降沿中断（10k↑3V3） |
| 36 | RXD0 | U0_RX | UART0 下载/调试 RX | 接下载器 TX |
| 37 | TXD0 | U0_TX | UART0 下载/调试 TX | 接下载器 RX |

**空闲/预留**：IO1(ADC1_CH0)、IO2(ADC1_CH1)、IO4(ADC1_CH3)、IO8、IO9。
**禁用**：IO35/36/37（PSRAM）。**strapping 保持默认**：IO0/IO3/IO45/IO46。

---

## 2. NCP81239 I²C 接口

- **7-bit 从机地址**：`0x74`（NCP81239；若为 NCP81239A 则 0x75。本板料号 NCP81239MNTXG = 0x74）
- **速率**：≤1 MHz（支持标准/快速/Fm+）
- **上拉**：SDA/SCL 各 4.7k 到 **3.3V**（比较器阈值 1V，3.3V 识别为 HIGH，无需电平转换）
- **写**：`[addr<<1|0] [reg] [data] [data+1] ...`（寄存器地址自增）
- **读**：先 pseudo-write 设寄存器指针，再 repeated-start 读

### 2.1 寄存器映射（Table 20）

| 地址 | 名称 | 类型 | Bit 说明 |
|:----:|:-----|:----:|:---------|
| 00h | EN / 控制 | R/W | [3]en_int [2]en_mask [1]en_pup [0]en_pol |
| 01h | dac_target | R/W | 8 位输出电压目标（配合 03h.bit4 共 9 位） |
| 02h | slew_rate | R/W | [1:0] 压摆率 |
| 03h | freq / dac_lsb | R/W | [4]dac_target_lsb [2:0]pwm_frequency |
| 04h | 功能使能 | R/W | [5]cs2_dchrg [4]cs1_dchrg [2]dead_battery_en [1]cfet [0]pfet |
| 05h | 电流限 | R/W | [5:4]ocp_clim_neg(CLIN) [1:0]ocp_clim_pos(CLIP) |
| 06h | CLIND 比较器 | R/W | [3:2]cs2_clind [1:0]cs1_clind |
| 07h | gm 补偿 | R/W | [7]gm_amp_config [6:4]hi_gm [3]gm_manual [2:0]lo_gm |
| 08h | ADC 控制 | R/W | [5]dis_adc [4:2]amux_sel [1:0]amux_trigger |
| 09h | 中断屏蔽 | R/W | [7]i2c_ack [6]vchn [4]tsd [3]pg [2]ocp_p [1]ov [0]clind |
| 0Ah | 中断屏蔽2 | R/W | [0]int_mask_shutdown |
| 10h | vfb | RO | 7 位输出电压 ADC 回读 |
| 11h | vin | RO | 7 位输入电压 ADC 回读 |
| 12h | cs2 | RO | 7 位输出电流 ADC 回读 |
| 13h | cs1 | RO | 7 位输入电流 ADC 回读 |
| 14h | 状态/中断源 | RO | [7]i2c_ack [6]vchn [4]tsd [3]pg_int [2]ocp_p [1]ov [0]ext_clind_ocp |
| 15h | shut_down | RO | [0]shut_down |

---

## 3. 输出电压设定（本板核心公式）

**原理**：环路把 FB 拉到内部 DAC 的参考电压 V_DAC，故 `Vout = V_DAC × DIV`，DIV=7.847。

```
V_DAC(mV) = Vout(V) / 7.847 × 1000
dac_code  = round(V_DAC / 10)          // 10mV 步进（默认）
写 01h = dac_code (8 位)               // 5mV 精档时另置 03h.bit4=dac_target_lsb
```

- **DAC 范围**：0.1V~2.55V（step 10mV，或 5mV 带 lsb），对应本板 **Vout 0.78V ~ 20.01V**
- **每步分辨率**：10mV V_DAC → **78.5 mV Vout**；用 5mV lsb → **39.2 mV Vout**
- ⚠️ **禁止写 0V**（手册明确）；要 0V 输出请拉低 P_EN(IO21) 或写 00h 禁用

**换算速查表**（10mV 步进）：

| 目标 Vout | V_DAC | 写 01h | 实得 |
|:--------:|:-----:|:------:|:----:|
| 0.78V(最低) | 99mV | 0x0A | 0.78V |
| 1V | 127mV | 0x0D | 1.02V |
| 5V | 637mV | 0x40 | 5.02V |
| 12V | 1529mV | 0x99 | 12.01V |
| 15V | 1912mV | 0xBF | 14.99V |
| 20V(最高) | 2549mV | 0xFF | 20.01V |

> DAC 0x00~0x09 为 Reserved（<0.1V 禁用区），有效码从 **0x0A** 起。

---

## 4. 限流设定（本板 RSENSE = 2mΩ）

> ⚠️ 手册 Table 9 的电流列是按 RSENSE=5mΩ 标的，本板是 **2mΩ**，须按 `I = V_threshold / 2mΩ` 重算（见下表）。

### 4.1 正电流限 CLIP（05h [1:0]，输入侧 CSP1-CSN1）

| CLIP[1:0] | 阈值 | 本板触发电流(2mΩ) |
|:---------:|:----:|:----------------:|
| 00 | 38mV(默认) | 19A |
| 01 | 23mV | 11.5A |
| 10 | 11mV | 5.5A |
| **11** | **70mV** | **35A** ← 本板必须设这个 |

> ⚠️ **必设 CLIP=11**：默认 38mV→19A，低于 Boost 正常峰值电流 27.3A，会误触发限流。见 note.md 风险#4。

### 4.2 负电流限 CLIN（05h [5:4]，输出侧 CSP2-CSN2）

| CLIN[1:0] | 阈值 | 本板触发电流(2mΩ) |
|:---------:|:----:|:----------------:|
| **00** | **-40mV(默认)** | **-20A** ← 保持默认即可 |
| 01 | -25mV | -12.5A |
| 10 | -15mV | -7.5A |
| 11 | 0mV | 0A |

→ **05h 写 0x03**（CLIN=00 + CLIP=11）。

### 4.3 CLIND 外部比较器（06h）

本板 CLIND 引脚**悬空未用**，应在 09h 屏蔽其中断（见初始化）。如启用：阈值 0.25/0.75/1.5/2.5V（00/01/10/11），本板 R_CS=8.2k、RSENSE=2mΩ 下默认 0.25V ≈ 3.0A。

---

## 5. 监测量回读（ADC，7-bit，20mV/LSB）

先写 08h 选通道(amux_sel)+触发(amux_trigger)，再读对应 RO 寄存器。本板换算：

| 读寄存器 | amux_sel(08h[4:2]) | 物理量 | 换算公式 | 分辨率 |
|:--------:|:------------------:|:------:|:--------|:------:|
| 10h vfb | 000 | 输出电压 | `Vout = code × 20mV × 7.847 = code × 156.9mV` | 157mV |
| 11h vin | 001 | 输入电压 | `Vin = code × 20mV × 10 = code × 200mV` | 200mV |
| 12h cs2 | 010 | 输出电流 | `Iout = code × 20mV / (2mΩ×5mS×8.2k) = code × 0.244A` | 0.244A |
| 13h cs1 | 011 | 输入电流 | `Iin = code × 0.244A`（同上，满量程 ~31A） | 0.244A |

- CS 通道增益 = RSENSE × 5mS × R_CS = 0.002 × 0.005 × 8200 = **0.082**；`I = V_CS / 0.082`
- **amux_trigger**（08h[1:0]）：00=故障时单次(默认)、01=单次读、10=连续读
- 建议：使能前先设 08h；改通道时先写 0 再写新值（手册要求）

> **输入功率/电流限制逻辑**（note.md 风险#8）：5525 接口只有 10~15A 能力，固件读 cs1(13h) 估算 Iin，超 15A 时降低 dac_target 限制 P_out≤150W。XT30 接口无此限制。区分接口靠人工/无自动检测。

---

## 6. 开关频率 / 压摆率 / gm

### 6.1 频率（03h [2:0]，pwm_frequency）
| 值 | 频率 | | 值 | 频率 |
|:--:|:----:|-|:--:|:----:|
| 000 | **600kHz(默认)** | | 100 | 750kHz |
| 001 | 150kHz | | 101 | 900kHz |
| 010 | 300kHz | | 110 | 1.2MHz |
| 011 | 450kHz | | 111 | 保留 |

> 本板设计频率 600kHz，保持默认。改频建议先禁用芯片再改，避免大电流下毛刺。

### 6.2 压摆率（02h [1:0]，soft-start / 动态调压速率，FB=0.1×V2 基准）
| 值 | 速率 |
|:--:|:----:|
| 00 | 0.6 mV/µs(默认,最慢) |
| 01 | 1.2 mV/µs |
| 10 | 2.4 mV/µs |
| 11 | 4.8 mV/µs |

### 6.3 gm 补偿（07h）
默认 500µS。COMP 硬件为初始值（R17 2k + VR1 5k 可调 + C34 47nF + C32 1nF），需硬件调试后定。gm 可在 87~1000µS 调（07h[6:4] hi_gm，需 gm_manual 配合），用于大容性负载变化时的环路增益。**初期保持默认，调试阶段再动。**

---

## 7. 推荐初始化序列

> 上电时 P_EN(IO21)=低，功率级禁用。MCU 启动后按序配置，最后再使能。

```
1. GPIO 初始化：P_EN(IO21) 输出低；INT(IO42) 输入+下降沿中断
2. I²C 初始化 @0x74
3. 写 05h = 0x03          // CLIP=11(70mV/35A) + CLIN=00(-40mV/-20A)  ★必须
4. 写 03h = 0x00          // 600kHz + dac_lsb=0（10mV步进）
5. 写 02h = 0x00          // 压摆率 0.6mV/µs（或按需加快）
6. 写 01h = <dac_code>    // 初始输出电压，如 5V=0x40；禁止 0x00
7. 写 08h = <amux_sel|trigger>  // 使能前配置 ADC，如连续读输出电压
8. 写 09h = 0x01          // 屏蔽 CLIND 中断（本板 CLIND 悬空）★
9. 写 04h：dead_battery_en=0, cfet=0, pfet=0   // 死电池/PD 功能全关
10. 拉高 P_EN(IO21)       // 使能功率级，芯片按 slew 软启动到 01h 目标
   （EN 拉高后延时再解除对 CLIND 的忽略——见坑#5）
```

调压运行时：只需改写 **01h**（+ 需要时 03h.bit4）。切勿写 0x00。

---

## 8. NTC 测温（3 路外置，10kΩ@25°C B3950）

⚠️ **上拉分选，每路换算公式不同**。分压：`3V3 ─[Rpull]─┬─ ADC ─[NTC]─ GND`，读到 `V_adc` 后：
```
R_ntc = Rpull × V_adc / (3.3 − V_adc)
T = 1 / ( 1/298.15 + ln(R_ntc/10000)/3950 ) − 273.15   // B 参数法
```

| 网络 | 座子 | ADC 脚 | Rpull | 适用量程 | 用途 |
|:----:|:----:|:------:|:-----:|:--------:|:----|
| NTC4_ADC | NTC_N | IO5 | **3.3k** | 10~100°C | 通用/功率区测温 |
| NTC2_ADC | NTC_L1 | IO6 | **33k** | -20~+20°C | 低温段 |
| NTC3_ADC | NTC_L2 | IO7 | **33k** | -20~+20°C | 低温段 |

- **⚠️ 别信网络名里的数字**（NTC2/3/4 与物理位置被 Layout 打乱），**以 Rpull 值判量程**：3.3k=通用(IO5)、两路 33k=低温(IO6/IO7)。
- ADC1 全部通道，12dB 衰减，采样 0~3.0V
- **必须固件校准**：ESP32 ADC 基准是内部带隙，分压却挂 AMS1117 的 3.3V，按实测 3V3 值算 R_ntc；多次采样平均
- 座子未插 NTC 时读数会跑到轨（开路→上拉把 ADC 拉到 3.3V），固件需判断"未接传感器"
- 功率区无板载 NTC（已删）：过温保护主要靠 **NCP81239 内部热关断 151°C** + 风扇策略

## 9. 风扇（4-pin PWM，Intel 规范）

- **FAN1**：PWM=IO38、TACH=IO40（管 Q7 2N7002 开漏）
- **FAN2**：PWM=IO39、TACH=IO41（管 Q8）
- PWM 频率 **25kHz**，TACH 上拉到 3V3、用 PCNT 读转速（4-pin 常通，转速可靠）

## 10. 拨轮开关（U6 = QS-302 五向拨轮，本板用 3 向）

- **KEY1=IO15**（U6.1）、**KEY3=IO16**（U6.2）、**KEY2=IO48**（U6.T 按下）
- active-low（按下=低），硬件有 RC 去抖（10k↑ + 100nF），**固件仍需软件去抖** 20~50ms
- 固件语义建议：一向=减、另一向=加、按下=确认/进菜单（左右向对应 KEY1/KEY3 需上电实测确定方向）

## 11. TFT 显示（ST7789V2，SPI 单向写）

- SPI2(FSPI)：CLK=IO12、MOSI=IO11、CS=IO13、DC=IO14；**RST=IO10**（独立，可单独硬复位屏）
- 无 MISO（只写不读）；逻辑 3.3V 直连
- **背光 BL=IO47**：LEDC **正相**（高=亮，与风扇相反），~1kHz
  - ⚠️ **常态占空比压到 30~50%**：背光挂 3V3 经 78M05，满亮会使 78M05 耗散偏高（裕量从 48% 掉到 23%），用 PWM 降平均电流

---

## 12. 固件已知坑（务必逐条注意）

1. **NCP 输出禁止写 0V**：dac_target=0x00 是 Reserved/禁用区，要 0V 拉低 P_EN 或写 00h 禁用，别用 01h=0。
2. **CLIP 必须设 11(70mV)**：否则默认 19A 会在 Boost 正常峰值(27.3A)误触发限流。
3. **风扇 PWM 逻辑反相**：MCU 输出高→NMOS 导通→Pin4 拉低→风扇 **0%** 占空比。占空比需**取反**（或定时器反相输出）。且只能开漏拉低，禁止推挽输出高。
4. **背光 PWM 正相**（高=亮）——与风扇相反，别写反了。
5. **CLIND 上电抖动**：EN 刚拉高时内部模拟未稳，CLIND 可能乱跳。本板 CLIND 悬空，**09h 写 int_mask_clind=1 屏蔽**；若用到需在 EN 拉高后延时再释放。
6. **改频先禁用**：pwm_frequency 改动建议先禁 P_EN 再改，避免大电流毛刺。
7. **软启动 3.3ms**：数字核复位需 ≥3.3ms，输出降到稳态后至少等 3.3ms 再重启软启动。
8. **NTC 三路换算公式不同**：按各自 Rpull（3.3k/33k/33k），不能套一个公式。
9. **ADC 全走 ADC1**：WiFi 开时 ADC2 不可用。
10. **5525 口固件限流**：读 cs1(13h) 估 Iin，5525 模式限 P_out≤150W（XT30 免）。
11. **strapping/PSRAM 脚**：IO0/3/45/46 保持默认；IO35/36/37 绝不使用。
12. **I²C 地址 0x74**（NCP81239，非 A 版）。

---

## 13. 关键器件位号速查（固件调试对照原理图）

| 功能 | 位号 | 值/型号 |
|:----|:----:|:-------|
| 电源控制器 | U2 | NCP81239MNTXG (I²C 0x74) |
| 主控 | U5 | ESP32-S3-WROOM-1-N16R8 |
| RSENSE1/2 | R1/R2 | 2mΩ |
| FB 分压 | R7(470Ω)+R4(68k) 串 / R6(10k) | DIV=7.847 |
| R_CS1/CS2 | R19/R14 | 8.2k |
| I²C 上拉 | R24(SDA)/R27(SCL) | 4.7k→3V3 |
| INT 上拉 | R28 | 10k→3V3 |
| NCP EN 下拉 | R18 | 10k→AGND |
| 拨轮 | U6 | QS-302-AGS8P |
| TFT 座 | FPC1 | AFC01-S10FCA-00 |
| 保险丝座 | FH1 | XF-505P（Mini 汽车保险丝，≤30A） |

> 完整位号对照表、硬件设计依据见 `note.md` 第〇章及各章。

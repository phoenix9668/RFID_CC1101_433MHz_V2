# V3.8 开发会话交接与跨电脑迁移

整理日期：2026-09-26。用途：让另一台电脑及新的开发会话接续本项目，不必重新推测历史决定。

本文件总结截至业务提交 `1fbb2580ae0ed49cf7788cf9818d0a23fe4ac6b5` 的状态。本文件本身属于后续纯文档提交，不是新的固件版本。9 月 26 日只进行了仓库、文件和主机测试检查，没有连接、烧写或复位板卡。

## 1. 先读结论

- 当前开发分支为 `codex/v38-rebuild-from-34d57c2`，不要误从 `main` 或喘息算法分支开始。
- Keil 到 GNU Arm GCC/CMake 的迁移、ADI 官方 ADXL362 驱动接入、CC1101 边界重构、事件调度和 EEPROM 日志存储已经完成。
- **当前源码已实现方案二：传感器设 50 Hz 档，按实测速率重建时间并滤波重采样为 25 Hz，再送入原六类行为算法。**
- **方案二尚未烧到板上。软件测试、历史数据模拟回放不等于真实 50 Hz 采集验收。**
- 最后有证据证明留在板上的，是原先 25 Hz 档位、正常 STOP 时序的 Diagnostics 固件。不是外部时钟实验固件，也不是当前重采样候选版。
- 六类协议和分类阈值未改。爬跨、反刍在本轮 Excel 对照中没有覆盖，不能宣称全部达标。
- 仓库只剩原项目目录一个 worktree；以后在这个目录用分支开发，不再默认创建额外 worktree。
- 源码、报告和 18 个 LFS 数据对象已推送。`.local/` 中的设备备份、日志和旧固件必须另行私密迁移，单独 clone 得不到它们。

### 文档优先级

`V38_REBUILD.md`、`HARDWARE.md`、`VERIFICATION.md` 是逐阶段形成的材料，仍保留早期的“25 Hz / 约 6 秒 / 不重采样”等历史说明以及旧构建体积。它们不代表当前源码仍然采用这些设置。

当前采样与算法链路以本交接、[RESAMPLING.md](RESAMPLING.md)、[方案二验证报告](../ADXL362_Resampling_Validation/报告.md) 和实际代码为准。硬件历史事实继续查阅 `VERIFICATION.md`、`ADXL362_MEASUREMENTS.md`，不要把历史上板结果套用到新固件。

## 2. Git 状态与提交脉络

- 远端：`https://github.com/phoenix9668/RFID_CC1101_433MHz_V2.git`
- 指定重建基座：`34d57c2a93b57979fecb929067e49df47c17c14d`。
- 当前业务 HEAD：`1fbb2580ae0ed49cf7788cf9818d0a23fe4ac6b5`。
- 上次已在线核对该提交到达远端，LFS 上传为 `18/18` 完成。9 月 26 日开始交接整理前，工作区干净，本地分支与远端跟踪引用一致。
- `main`、`panting`、`claude/panting-branch-setup-6cb67d` 等分支保留，未把它们的后续算法混入本次从指定基座进行的重建。

| 提交 | 内容 |
| --- | --- |
| `22fc32b` | 指定基座的行为算法、协议及主机回归基线 |
| `dba14b4` | ADI no-OS 接入、STM32 适配、TI 参考无线驱动 |
| `cffc98b` | GCC/CMake、事件循环、STOP、EEPROM/调度运行时 |
| `89c5870` | ST VS Code 开发流程与初轮验收文档 |
| `bf17805` | 硬件导出资料保持原始字节 |
| `0036a5c`、`d4a86a7` | DSLogic 准备、FIFO/SPI 与启动配置抓波 |
| `ced6af8` | DATA_READY 实际采样节奏实验 |
| `7bc8f49` | 25/50/100 Hz 档位分档实验 |
| `3c02631` | 外部参考时钟 A/B 对照实验 |
| `0d538ed` | 方案二重采样、模拟回放与逐类门槛 |
| `a15a9a9` | 供货商调查报告、PDF、单工作区流程 |
| `1fbb258` | LFS 数据整理、ignore、小型历史验证证据 |

删除的是临时 worktree 的检出目录，不是上述分支或提交。旧工作区所有本地文件已归档并校验；保留目录为 `.local/worktree-archive/v38-rebuild-20260922/`。

## 3. 用户要求与不可改变的边界

设备是牛颈部佩戴的 V3.8 项圈。原设计每约 6 秒取一批 ADXL362 FIFO，将三轴数据经算法输出每秒一个六类行为，累计 20 分钟，开启 CC1101 发往原基站；维护 12 个历史窗口，并用内部 EEPROM 防意外重启丢失统计。生产模式重视低功耗。

后续工作继续遵守：

1. 无线报文格式、字段偏移、大小端和 CRC 不变，必须兼容原基站。
2. 保留原六类算法的整数运算、阈值、历史修正和输出延迟，不能擅自合并后续喘息分类实现。
3. 不增加 ACK、无限重试或生产原始三轴无线调试包。
4. 正常生产关闭 UART 日志、非必要 LED；Diagnostics 才有诊断输出。
5. 约 60 秒检查点损失目标不涵盖 EEPROM 故障、未读 FIFO、算法预热等全部情况，不能承诺绝对最多丢 60 秒。
6. 不擦除身份数据、不整片擦除、不改变 RDP/选项字节，不用解除保护解决连接失败。
7. 新方案提高采集档位后允许更频繁读取 FIFO；不能机械保留 6 秒等待导致 FIFO 溢出。
8. 用户采用方案二，不采用低速 19.75 Hz 数据直接上采样作为生产默认；MCU 持续输出外部时钟仅用于台架定位，不作为当前 STOP 低功耗生产方案。

## 4. 硬件、驱动与模块

### 硬件

硬件原始导出位于 `PCB/RFID_CC1101_433MHz_V3.8/`。部分图框仍有旧版标签，不能据此随意改动原始文件。

| 项目 | 当前约定 |
| --- | --- |
| MCU | STM32L051C8T6，64 KiB Flash / 8 KiB RAM / 2 KiB 内部数据 EEPROM |
| 时钟 | HSE 12 MHz、LSE 32768 Hz、系统 32 MHz |
| 加速度计 | ADXL362，SPI2，4 MHz |
| 无线 | E07-433M20S / CC1101，SPI1，4 MHz |
| 电源 | 电池与 LP5907-3.0；TP4056 充电 |
| ADC | PA0，10 MΩ/10 MΩ 分压，C12 100 nF，电压精度/建立时间仍需测量 |
| UART | USART1，PA9=TX、PA10=RX，Diagnostics 为 115200 8N1 |
| 调试 | SWD 必须考虑 NRST；STOP 下可能需要 connect-under-reset |

### 驱动来源

- `Drivers/ADI/adxl362.c/.h` 是 ADI no-OS 固定提交 `0c4ef5cd278555ba4d426b35f87f30102460361f` 的上游原文件，版权头和哈希有记录。
- `Platform/Src/no_os_port.c` 是固定内存适配层，并非完整 no-OS 移植；错误不能被当作有效传感器数据。
- 官方 FIFO helper 有 512 字节栈缓冲，所以每次最多读 510 字节有效载荷，且保持偶数：900=`510+390`，1024=`510+510+4`。
- `Platform/Src/radio.c` 是项目自己的 **TI 官方资料参考实现**，不是直接集成的 TI 官方驱动。SWRC021 下载当时返回 403，未取得或审阅示例源码，不得声称已移植该源码。
- RF 47 项配置、PA 起始值 `0xC0`、地址 `0xEF`、同步字 `0x1234` 和 60 字节补 FIFO 逻辑保持兼容。
- HAL/LL/CMSIS 保留 STM32CubeL0 V1.12.2；没有因为使用 CMake 把 LL 全部改成 HAL。
- `Drivers/PCG` 为有许可证的 PCG Minimal C，用于非安全随机填充，不能用于认证或加密。

完整来源与文件哈希：[SOURCES.md](SOURCES.md)。

### 架构入口

| 文件/目录 | 责任 |
| --- | --- |
| `Core/` | CubeMX 生成的外设初始化、ST 系统代码、启动文件 |
| `Platform/Src/entry.c`、`interrupts.c` | 实际入口和中断，不把重新生成的 Core/main.c 重复编入 |
| `Platform/Src/board.c`、`clock.c` | 有期限的总线访问、外设启停、RTC/STOP、EEPROM 硬件接口 |
| `Platform/Src/sensor.c` | ADXL362 配置/回读/FIFO，官方调用包装 |
| `Platform/Src/radio.c` | CC1101 事务、有限状态收发、超时与关电 |
| `App/Src/fifo_parser.c` | 按轴标签组装完整 XYZ，丢失/错位不伪造样本 |
| `App/Src/sample_clock.c` | RTC 与 FIFO 数据量构建采样时间 |
| `App/Src/resampler.c` | 固定内存整数滤波与 25 Hz 时间网格输出 |
| `App/Src/behavior.c` | 原始六类行为分类器，内部历史状态 |
| `App/Src/history.c` | 六类计数、12 窗口 |
| `App/Src/protocol.c` | 191 字节协议与 CRC，可脱离硬件测试 |
| `App/Src/storage.c` | EEPROM 日志、掉电恢复、旧布局一次性迁移 |
| `App/Src/app.c` | 原子领取中断事件、优先采集、检查点、窗口/发送调度 |
| `Tests/legacy/` | 精确基座的测试参照，不属于生产目标 |

中断只记录事件，主循环处理 SPI、算法和存储。STOP 前原子检查待处理事件。无线关闭顺序先处理 PA/LNA、中断和引脚，再关模块电源，避免反向供电。

## 5. 无线与 EEPROM 合同

应用载荷固定 **191 字节**。无线 FIFO 的长度字段为 192，另含地址 `EF`；硬件包 CRC 和应用 CRC 均保留。

| 偏移 | 字节数 | 含义 |
| --- | ---: | --- |
| 0 | 6 | 设备号，原字节顺序 |
| 6 | 32 | 非安全随机填充 |
| 38 | 144 | 六类各 12 个 uint16 历史计数，大端，类别优先排列 |
| 182 | 1 | 封存上一窗口后的当前/下一历史槽 |
| 183 | 2 | 原始电池 ADC 值，大端，不是 mV |
| 185 | 2 | 复位计数，大端 |
| 187 | 4 | 前 187 字节的 CRC32，大端 |

CRC 为 reflected IEEE CRC32，poly=`EDB88320`，init=`FFFFFFFF`，xorout=`FFFFFFFF`。先封存统计/发送快照，再进入下一个窗口；发送失败关闭无线，历史留待后续周期携带。

EEPROM 基址 `0x08080000`，旧区 `0x000..0x23F` 不改写。新区 `0x240..0x7FF` 为 16 个历史槽 + 30 个当前窗口检查点槽，每槽 32 字节，含序号、六计数、时间进度、复位次数、格式信息、CRC 和最后写入的提交标记。先使旧提交字无效，写内容，再写提交字并回读校验。

启动选择最新有效记录。旧数据曾出现进度 252、窗口总数约 2832；用户明确同意“保留设备号和旧区原值，非法统计按未知开启新窗口”。不长期双写旧区。写入失败后本次运行停止继续写日志，保留上一有效记录，RAM 业务继续。

磨损预算仍是风险：当前提交字在槽复用时可能经历两次写周期，不能宣称长期寿命已经最优。后续如优化，必须单独评审存储格式和兼容性。详见 [PROTOCOL_STORAGE.md](PROTOCOL_STORAGE.md)。

## 6. 25 Hz 偏慢问题的定位过程

用户历史观察：有的项圈每小时六类统计总和接近 3600，有的只有约 2800 至不到 3000。本次板卡的证据与后者一致。

| 步骤 | 观察与结论边界 |
| --- | --- |
| 普通 STOP 与不休眠对照 | 都约 19.75 Hz；不是仅在 MCU STOP 时发生 |
| 50 s / 1 MHz 低速抓波 | 校正 D4 接触后，水位间隔约 7.642 s；1 MHz 只用于节奏，不能解码 4 MHz SPI |
| “一组 SPI 没有 INT2” | 检查点会主动读取部分 FIFO，不是每次 SPI 都必须由水位中断触发 |
| 50 MHz 高速 FIFO 抓波 | 状态 `07`，计数 `C5 01` 即 453 words；读取 `510+396` 字节，151 组 XYZ 标签连续，SCLK 约 4 MHz |
| 手动保持复位再释放的启动抓波 | 确认 ID、软复位、14 对配置写入/回读；`FILTER_CTL=51`、`POWER_CTL=02` 在真实 SPI 总线上成立 |
| DATA_READY 实验 | 988 个完整中断，平均周期约 50.631931 ms，即 19.750382 Hz |
| 25/50/100 Hz 分档 | 分别约 19.750956 / 39.502107 / 79.233530 至 79.269014 Hz，呈共同约 0.79 倍因子 |
| 外部时钟 A/B | 同一芯片内部约 19.746835 Hz，输入实测约 32000.9 Hz 外部参考后输出约 15.625 Hz |
| 50 MHz 外部时钟复核 | 一个 DATA_READY 周期内恰为 2048 个参考时钟周期，约 63.99814 ms，符合外部参考/分频关系 |

32 kHz 外部参考实验的预期值是约 15.625 Hz，不是 25 Hz；它用于对照时基，不能写成“已经用外部时钟得到 25 Hz”。外部时钟测试使用 MCU 定时器中断驱动 PB0，不能据此认为 MCU 在 STOP 中仍能持续提供该参考。

综合证据**强烈支持内部时基异常的工程判断**。用户认可按内部时钟问题处理。但这不等于 ADI 已确认芯片物理故障，更不能断言假芯片或仅凭万用表排除所有供电/边沿问题。C19 运行时万用表约 3.003 V、断电后缓慢放电的读数，不是高带宽电源纹波测量。没有正常项圈可现场对照，原基站也未就绪。

按 25 个样本输出一次分类，19.747 Hz 约只产生 2844 次/小时，解释了宏观计数缺口。早期抓波存在 D4 接错/接触问题及复位边界，最终结论使用的是后来有效抓波，而不是将所有文件一概视为有效。

详细图文证据：[供货商报告 Markdown](../ADXL362_25Hz_Investigation/README.md)、[最终 PDF](../../output/pdf/ADXL362_25Hz_Supplier_Report.pdf)、[测量记录](ADXL362_MEASUREMENTS.md)。不得把没有保存的 ADI 论坛交流猜写成官方确认。

## 7. 当前方案二实现

### 配置和时间处理

- 正常候选固件：`FILTER_CTL=0x52`，±4 g / 50 Hz / HALF_BW；`POWER_CTL=0x02`。
- 原活动/静止寄存器字节、水位 450 words、INTMAP/FIFO 配置保留。ODR 变化会改变活动检测的物理时间，但这组检测不驱动主分类 FIFO 路径。
- ODR/EXTCLK 专用实验仍以 `0x51` 为基线，且默认不开启；它们不是生产构建选项。
- FIFO 水位为 150 组三轴，正常 50 Hz 约 3 秒到水位，当前慢芯片约 39.5 Hz 时约 3.8 秒。不能等 6 秒才读，512-word FIFO 放不下。
- `sample_clock` 根据 RTC 毫秒差和实际读出 word 数重建批内均匀时间。它不是每个样本的硬件时间戳，仍需验证服务延迟/抖动。
- 首批数据只作锚点并丢弃，至少 2 秒建立初始估计，后续至少约 20 秒更新长周期速率观测。接受范围 32..65 Hz，并考虑一个样本的量化误差。
- 初始兜底读取期限 2.4 秒，之后约按 160 样本时间、最多 4.8 秒安排，覆盖水位 IRQ 丢失。突然大幅变速仍可能溢出，不能把它当作已消除的风险。

### 重采样和算法

- 固定 48 项 XYZ/时间环形缓冲，无 malloc，MCU 不使用浮点或三角函数。
- Kaiser 窗 sinc：8 Hz 截止、beta=5、±320 ms 支撑、1 ms 系数网格与系数插值；321 个 int16 半核查表，int64 累加和直流归一化。
- 8 Hz 是所用 `firwin` 定义的约 -6 dB 点，不等同于传感器原模拟滤波的 -3 dB 点。
- 在连续 40 ms 网格上输出 25 Hz XYZ，约增加 320 ms 未来样本支撑延迟。每 25 个有效输出交给原算法形成一次分类。
- 原算法阈值和历史修正未调参。统计按结果可输出的时间归属窗口，不回填已封存窗口。
- SPI 错误、FIFO 溢出、错误轴标签、时间断流均使相关状态重置并重新预热；不外推、补造或复制标签来强制凑满计数。
- 稳态仿真完整窗口为 1200 次；启动首窗口、故障恢复及尾部不足一秒的数据明确可能少计。

## 8. 离线数据、门槛与已知结果

数据来自 `Doc/MATLAB/` 的 4 个 Excel、5 个工作表，共 150,526 组三轴样本。用户确认采集配置为 25 Hz，但没有历史实际 ODR 测量、独立时间戳或人工行为标注。

保留原 XYZ 整数单位；没有自行缩放或调整佩戴方向。`example_data.xlsx` 的 Sheet2 有样本序号重置，拆成三个各 31 样本片段，不跨断点插值。39.5/50/57.5 Hz 输入是由历史 25 Hz 数据合成的敏感性实验，**不是新采集的真实 50 Hz 数据**，也不能恢复原带宽之外的信息。

以约 39.502 Hz 模拟输入，剔除预热和边界后，共 5934 个名义秒可比较，557 秒分类不同，总体一致率 90.61%。仅加相同滤波的 25 Hz 控制组有 555 秒不同；完整方案二与该控制组有 52 秒不同，说明本次差异主要与滤波有关，但不能据此认定哪种分类正确。

用户不要求总体 100% 一致，要求爬跨/活动各至少 95%，采食/反刍各至少 90%。目前工具保守地同时约束“相对原算法的召回率”和“相对原算法的精确率”；没有人工真值，不能称为识别准确率。

| 行为 | 门槛 | 原算法秒数 | 对照召回率 | 对照精确率 | 状态 |
| --- | ---: | ---: | ---: | ---: | --- |
| 爬跨 | 95% | 0 | 不可评估 | 不可评估 | 无覆盖，无法验收 |
| 活动（类别 3，原名运动） | 95% | 11 | 100% | 100% | 样本内达到，但样本太少 |
| 采食 | 90% | 3155 | 92.96% | 93.89% | 合并数据达到 |
| 反刍 | 90% | 0 | 不可评估 | 不可评估 | 无覆盖，无法验收 |

50/57.5 Hz 模拟组采食召回率均约 92.74%，精确率约 93.87%/93.81%。同一批数据的多个合成速率不能累计为独立样本。

**单文件例外：**`vofa_9.15.11.xlsx` 在约 39.5 Hz 组的采食召回率为 698/777=89.83%，精确率 698/774=90.18%；50 Hz 组召回率 699/777=89.96%。合并达标不等于每个文件都达标。11 秒活动也不等于 11 次独立行为事件。

`Tools/behavior_acceptance.py` 会重新核对 28 份配对 CSV 与转换矩阵，输出逐记录/分速率合并的 `acceptance.json`。零分母标为不可评估，不按 100% 处理；加 `--require-pass` 时，本批应返回 **2**，因为缺少爬跨/反刍覆盖。这是预期的验收未完成，不是程序崩溃。

已提交证据在 `Doc/ADXL362_Resampling_Validation/`；不要为了提升无标注一致率盲目拟合旧算法输出。

## 9. 已验证与尚未验证

### 软件

- 最新三个固件构建已在迁回后的原目录通过，没有新增编译/链接警告。
- 9 月 26 日复跑现有主机构建的 12 项 CTest，全部通过；复跑 6 项门槛测试、28 项波形/外部时钟分析测试，全部通过。
- LFS 完整性检查通过。整理时逐一确认了 18 个原文件、指针和本地 LFS 对象的 SHA-256 一致，文件内容未改。
- 四组 4 小时 20 分钟调度仿真覆盖慢速、50 Hz、65 Hz、FIFO IRQ 丢失；首窗口允许预热损失，后续 12 个完整窗口各有 1200 次分类。
- 原算法对基座的原三组 CSV 回归共 15,738 次输出一致；这是“分类器未改”的证据，不能替代方案二的重采样分类验收。
- 协议、RF 表及边界、FIFO 分块失败、存储断电写入阶段、复位及并发事件均有主机测试。

| 构建 | Flash | RAM，含 2048 B 预留栈 |
| --- | ---: | ---: |
| Debug | 25,188 B | 5,160 B |
| Release | 22,872 B | 5,152 B |
| Diagnostics | 29,196 B | 5,568 B |

这些是当前方案二的构建值，不要用旧 `VERIFICATION.md` 的早期值覆盖。单函数静态栈报告不是完整调用链或上板高水位；CPU 时间和功耗没有因此完成验收。

### 历史上板证据

已看到 ADXL362 ID/配置、正常 FIFO、CC1101 PARTNUM=00/VERSION=14、关键 RF 回读和一份 `tx-ok`；曾跨过恢复后窗口边界，确认检查点与复位恢复。基站不在现场，**tx-ok 不等于基站收到/解码成功**。没有连续 4h20 实机验收，也没有新方案的真实功耗结果。

### 最后有证据的板上版本

外部时钟实验后，恢复并校验的是原 26,216 B Flash 的正常 Diagnostics 镜像，最后记录为 stage=7、elapsed=900、reset=53、errors=0/0/0。此处是历史记录，不是对今天板卡运行状态的实时确认。

该镜像 HEX SHA-256：

```text
6A823BA02515CF614E70F79850F2E62D5BDE56AD750DAEA67C706877982D6866
```

9 月 26 日在本地重新找到并核对该哈希的路径：

```text
.local/worktree-archive/v38-rebuild-20260922/build/Diagnostics-local/RFID_CC1101_V38.hex
.local/worktree-archive/v38-rebuild-20260922/build/final-clean/Diagnostics/RFID_CC1101_V38.hex
```

同时保留对应 ELF 和备份清单。不能用当前新编译的 Diagnostics 替代它并声称是“原样恢复”。

## 10. 新电脑迁移步骤

建议在新电脑建立独立 clone，使用 Git/Git LFS 传递受版本管理内容；不要把旧 `.git` 和 CMake 缓存直接覆盖到新检出，也不要两台电脑同时操作同一个被同步的 `.git`。

### 第一步：拉取正确分支与 LFS 数据

先安装 Git for Windows 和 Git LFS，然后在准备放项目的父目录执行：

```powershell
git clone --branch codex/v38-rebuild-from-34d57c2 https://github.com/phoenix9668/RFID_CC1101_433MHz_V2.git
Set-Location RFID_CC1101_433MHz_V2
git lfs install --local
git lfs pull
git lfs fsck
git status --short --branch
git lfs ls-files --size
```

旧机 Chrome 能访问 GitHub，但命令行直连超时。最后使用用户提供的 FLCash 本地 7890 端口完成推送；没有写入全局代理。新机如同样需要代理，请核对实际端口，临时用：

```powershell
git -c http.proxy=http://127.0.0.1:7890 lfs pull
git -c http.proxy=http://127.0.0.1:7890 fetch origin
```

首次 clone 也可在 `git` 后加同样的 `-c` 参数。不要关掉 TLS 校验或在仓库内保存凭据。GitHub 的 ZIP 源码包不能替代完整 Git/LFS 迁移；Excel、视频、功耗文件必须是实际内容，不是三行 pointer 文本。

### 第二步：另行私密迁移文件

以下文件被有意忽略，不会随 Git/LFS clone 获得：

| 内容 | 旧机项目内位置 | 处理 |
| --- | --- | --- |
| 原始 Flash/EEPROM/选项字节及清单 | `.local/backups/` | 必须保留完整文件夹、哈希和设备对应关系，不能公开上传 |
| UART、抓波副本、解码 JSON 等 | `.local/captures/` | 建议完整迁移，用于还原现场证据 |
| 其他 EEPROM 对照与运行记录 | `.local/` 根下的相关 bin/json/txt | 建议随整个 `.local/` 私密带走，勿只复制一个 Flash 文件 |
| 最后恢复到板上的精确镜像 | 上节列出的归档 build 目录 | 至少保留对应 HEX、ELF 和关联清单 |
| 旧 worktree 完整归档 | `.local/worktree-archive/v38-rebuild-20260922/` | 作为只读历史备份，不能直接用其中的 CMake 缓存构建 |
| 旧工具/本地配置备份 | 归档内 `primary-overlaps/` | 仅作参考，不应覆盖新机配置 |
| `CMakeUserPresets.json`、`.local/tools.json` | 项目根及 `.local/` | 记录旧设置即可，在新电脑重新生成路径 |

最简单的私密交接是另行备份整个 `.local/`，确认新机能读到全部文件后再决定是否删旧机内容。云同步目录需确认文件已实际下载，不能只看到占位文件名就认为备份完整。

归档中的 `local-branches.bundle` 是截至 `a15a9a9` 的旧应急快照，不含 `1fbb258` 及本交接提交，也不包含 LFS 对象，不能当作最新完整迁移包。`build/`、`tmp/` 中通常是可重建输出，不必整目录当作可运行环境搬过去。

### 第三步：重建开发环境

已验证的固件工具版本为 STM32CubeIDE for Visual Studio Code 3.10.0、STM32CubeCLT 1.18.0、GNU Arm GCC 13.3.1、CMake 3.28.1 和配套 Ninja。新装扩展如有不同版本，明确记录差异，不要让它静默切换编译器；本轮未采用 Clang 编译固件。

主机测试还需要原生 Windows GCC，不是 arm-none-eabi-gcc。旧机借用了 Vivado 中的 MinGW GCC 6.2.0；新机不必为了它安装整个 Vivado，可以配置可用的原生 GCC，再重新验证。

离线回放旧机环境已核对为 Python 3.12.14、NumPy 2.5.3、SciPy 1.18.1、openpyxl 3.1.5。推荐使用隔离环境，示例：

```powershell
python -m venv .local/venv
$python = (Resolve-Path .local/venv/Scripts/python.exe).Path
& $python -m pip install numpy==2.5.3 scipy==1.18.1 openpyxl==3.1.5

# 两个路径按新电脑实际安装位置替换。
$cubeClt = 'C:/ST/STM32CubeCLT_1.18.0'
$hostGcc = 'C:/your-native-gcc/bin/gcc.exe'
./Tools/setup-local.ps1 -CubeClt $cubeClt -HostGcc $hostGcc -Python $python
./Tools/open-vscode.ps1
```

此处故意暂不传 `-CsvData`，原因见下一小节。三个本地预设为 `Debug-local`、`Release-local`、`Diagnostics-local`。旧的索引配置和 ST bundle 锁文件已归档，避免沿用旧路径/不同工具链。CubeMX 再生成时只合并受控生成区，不覆盖 App、Platform 和根 CMake。

### 第四步：先软件验证，不自动烧板

```powershell
./Tools/verify.ps1 -BuildRoot build/migration-verify
& $python -B Tests/test_behavior_acceptance.py
& $python -B -m unittest discover -s Tools -p 'test_analyze_adxl362_*.py'

& $python -B Tools/validate-resampling.py --data-root Doc/MATLAB --host-build build/migration-verify/host --output .local/migration-replay
& $python -B Tools/behavior_acceptance.py --results .local/migration-replay/results.json --require-pass
```

最后一条在当前数据集上预期退出码为 2，原因是类别覆盖不足。输出到 `.local/migration-replay` 是为了不覆盖已提交证据。新构建 ELF 包含新路径，文件哈希可能不同，不应只凭 ELF 哈希变化认定功能变了。

### 早期三份 CSV 回归数据的特殊情况

`Tools/behavior-csv.py` 期待 `chuanxi.csv`、`喘息20250820 1644.csv`、`02 1004 20230315 1819-2219 29W.csv`。这三份原始文件**不在当前分支目录中**，不能把当前 `*_result.csv` 当成它们。

原始数据仍存在旧分支提交 `9bfd24a975ab8d8865f7d87dc3e25ba1834e76aa` 的 `Doc/data/`。9 月 26 日已确认，通过 Git 的 checkout 过滤器恢复后，三份文件的 SHA-256 均与 `Doc/Development/csv-regression.json` 一致；Git blob 的 LF 换行字节与旧 Windows 文件不同，所以直接 archive 导出不保证同样的文件哈希。

在新机需要这组回归时，可只导出这三个数据文件到 `.local/legacy-csv/`，核对上述哈希后再给 `setup-local.ps1` 传入该目录。不要为取数据切换并合并旧分支实现，也不要把旧 `.local/tools.json` 中失效的 `Doc/data` 路径直接沿用。只做当前重采样验证时，现有 LFS Excel 已足够复现本轮结果。

### 第五步：准备下一轮硬件验证

旧机串口是 COM27，新机需重新枚举，不能硬套。ST-Link 序列号留在私密备份，不写入共享配置。先核对同一设备及备份清单，再只读备份最新 EEPROM/选项字节；板卡在等待期间可能继续产生新统计，不能用早期备份替代最新烧写前快照。

ST 官方调试适配器为 `stlinkgdbtarget`。提供“构建/烧写/调试”和“仅符号、不烧写不复位”两种入口，后者仍可能暂停 CPU。ST GDB server 7.10.0 曾被发现监听所有接口，`serverHost=localhost` 不是服务端绑定限制；`Tools/debug-network.ps1` 用专门的防火墙限制做防护，进一步交互调试前需在新电脑安装并验证。正常电流测试须断开 SWD/UART 干扰，不保留冻结看门狗或低功耗调试设置。

## 11. 逻辑分析仪接线和采集经验

用户设备为 DSLogic U3Pro16，旧机 DSView 1.3.2。接线要断电操作，共地，不把分析仪电源接入板卡电源；PB0 在外部时钟实验中由 MCU 驱动，不能同时接其他信号源。

| 通道 | 连接 |
| --- | --- |
| D0 | PB12 / ADXL362 CS |
| D1 | PB13 / SCK |
| D2 | PB15 / MOSI |
| D3 | PB14 / MISO |
| D4 | PB1 / ADXL362 INT2 |
| D5 | TP18 / PB0 / ADXL362 INT1，外部时钟实验用 |
| GND | 板卡 GND |

长时节奏：Stream、1 MHz / 50 s、1.5 V、无滤波、内部采样时钟，单次；用于 IRQ 间隔，不用于解码 4 MHz SPI。

SPI：Buffer、50 MHz / 100 ms、D4 上升沿、10% 触发位置。启动：50 MHz / 500 ms、D0 下降沿，先持续按住板卡复位、分析仪等待触发后再松开。外部时钟高速检查：50 MHz / 100 ms、D5 上升沿。特别核对 ms 与 us，曾有 100 us/100 ms 配置混淆。

`.dsl` 可以直接由仓库工具解析，无需强求 VCD 或重新导出 CSV。每次采集另存，保留原始文件及哈希，不覆盖既有证据。

## 12. 下一步优先顺序

1. 新电脑拉取正确分支和全部 LFS，对照本交接运行软件测试；确认私密备份与精确回退镜像已带到新机。
2. 用户确认硬件连接后，只读备份，准备安全烧写方案二 Diagnostics。切勿把已通过旧固件的寄存器/FIFO测试视为新方案已通过。
3. 采集真正 50 Hz 档位 XYZ 与 RTC/逻辑分析仪时间，验证速率估计、40 ms 输出网格、STOP 唤醒、兜底读取、20 分钟边界及无溢出。
4. 验证真实 20 分钟与至少 4h20 连续运行、复位/掉电恢复、异常重试、运行时栈高水位；窗口故障数据必须明确缺失，不伪造行为。
5. 补充爬跨/活动/反刍覆盖，最好人工标注；针对单文件采食 89.83% 先分析差异，不盲调阈值。早期 CSV 中有原算法反刍输出，但尚未纳入本轮方案二评估，可在确认数据使用范围后补充。
6. 原基站就绪后核对真实收包及全部字段/CRC，补短包与错误路径；不能只凭项圈 tx-ok 结案。
7. 使用独立运行的 Release 测休眠电流、采集电荷、发送电荷和 20 分钟总电荷；对比等功能基线，量化 50 Hz 档位加滤波的耗电代价。

剩余风险还包括：真实加速度模拟带宽与软件滤波不完全等价、历史实际采样率未知、极小活动样本、不完整类别覆盖、ADC 高阻分压精度、EEPROM 磨损预算，以及 ST VS Code GUI 的索引/调试验证尚不能仅凭命令行构建宣称完成。

## 13. Git 与资料管理注意事项

- 18 份 LFS 文件共 246,501,819 字节，包括 8 DSL、1 IOTPL、3 视频、4 Excel、2 大型回放 CSV；小型 CSV/JSON 和最终报告仍在普通 Git 中。
- 已确认重复的 precommit CSV、重复 report artifact、HTML 预览及一次性文档工具按精确路径忽略，没有删除原文件。
- 不要泛化忽略所有 CSV、JSON、Python、Doc 或 output，否则会隐藏真正的证据和工具。
- 历史喘息验证 Python/SQL 是档案，不是当前固件算法；其中旧会话绝对路径尚未做跨机器移植。
- 旧仓库存在一条损坏的 `refs/codex/turn-diffs/...` 检查点引用，会触发自动维护报错。此前用每命令 `-c maintenance.auto=false` 完成提交，没有删除未知引用；当前开发分支对象遍历无缺失且推送成功。新电脑采用全新 clone 可避免直接搬入该本地引用。
- 保持单 worktree；新任务可用 `git switch -c codex/<任务名>`。不要重新创建已删除的临时 worktree，也不要因分支名旧而强制改名/重建。
- 本会话用户已明确表达：要求“提交推送”时要完成远端核验，不能停在暂存。对私密硬件备份则不得擅自上传。

## 14. 建议阅读顺序

1. 本交接及 [NEXT_SESSION_PROMPT.md](NEXT_SESSION_PROMPT.md)。
2. [RESAMPLING.md](RESAMPLING.md)、[方案二验证报告](../ADXL362_Resampling_Validation/报告.md)、其 `acceptance.json`。
3. `App/Src/app.c`、`sample_clock.c`、`resampler.c`、`behavior.c` 及对应测试。
4. [PROTOCOL_STORAGE.md](PROTOCOL_STORAGE.md)、[SOURCES.md](SOURCES.md)、[HARDWARE.md](HARDWARE.md)。
5. [供货商报告](../ADXL362_25Hz_Investigation/README.md)、[ADXL362_MEASUREMENTS.md](ADXL362_MEASUREMENTS.md)、[VERIFICATION.md](VERIFICATION.md)，注意历史阶段。
6. [DATA_STORAGE.md](DATA_STORAGE.md)、[WORKFLOW.md](WORKFLOW.md)、`Tools/setup-local.ps1`、`Tools/verify.ps1`。

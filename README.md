# STM32 双轴云台实验

基于 STM32F407IGH6、BMI088 和 GM6020 的 Yaw / Pitch 云台控制工程，包含姿态解算、角度与速度 PID、CAN 电机通信和 VOFA+ 串口调试。另附 OpenMV 人脸检测示例。

## 工程入口

| 路径 | 内容 |
| --- | --- |
| [Core/Src/main.c](Core/Src/main.c) | 初始化、5 ms 控制循环、串口调参命令 |
| [modules/gimbal](modules/gimbal) | 电机控制与 PID |
| [modules/imu](modules/imu) | BMI088 驱动、AHRS 姿态解算、VOFA+ 通信 |
| [Makefile](Makefile) | GNU Arm 构建与烧录目标 |
| [openocd.cfg](openocd.cfg) | 当前 CMSIS-DAP / SWD 下载器配置 |
| [openmv/face_detect/main.py](openmv/face_detect/main.py) | 在 OpenMV 上运行的人脸检测示例 |
| [DEBUG_LOG.md](DEBUG_LOG.md) | 历史调试记录 |
| [文档](文档) | 开发板、电机和相机参考资料 |

## 当前代码中的硬件配置

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F407IGH6，Cortex-M4F |
| 时钟 | 外部晶振 12 MHz，系统时钟 168 MHz |
| IMU | BMI088，通过 SPI1 读取 |
| 电机 | CAN1，1 Mbps；Pitch ID 1，Yaw ID 5 |
| 调试串口 | USART6，921600 baud；TX PG14，RX PG9 |
| 下载器 | CMSIS-DAP，VID/PID `faed:4870`，SWD 500 kHz |

以上配置来自当前源码；换开发板、下载器或电机 ID 时，应同步核对对应初始化代码。

## 编译

需要 GNU Make 和 GNU Arm Embedded 工具链，确保 `arm-none-eabi-gcc`、`arm-none-eabi-objcopy`、`arm-none-eabi-size` 可从终端调用。

```bash
git clone https://github.com/czself/gimbal.git
cd gimbal
make -j
```

产物为 `build/demo1_imu.elf`、`build/demo1_imu.hex` 和 `build/demo1_imu.bin`。清理构建产物使用 `make clean`。

## 烧录与调试

连接与 [openocd.cfg](openocd.cfg) 匹配的下载器，并安装 OpenOCD 后：

```bash
make flash
```

该命令会烧录、校验并复位 MCU，复位后固件开始执行云台控制。上电调试前应固定机构并确认电机 ID、方向和限幅与实物一致。

VOFA+ 的发送与接收协议见 [modules/imu/vofa](modules/imu/vofa)，具体调参命令见 `main.c`。OpenMV 示例需在 OpenMV 的 MicroPython 环境运行，独立于 STM32 的 Makefile 构建。

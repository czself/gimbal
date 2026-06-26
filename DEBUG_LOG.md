# 调试错误总结

## 零、烧录 / OpenOCD 错误

### Bug 0: Unable to reset target ⭐⭐
- **现象**：`make flash` 时报错 `timed out while waiting for target halted` / `Unable to reset target`
- **原因**：`reset_config srst_only` 要求通过 SRST 硬件复位引脚复位芯片，但 RoboMaster C 板 SWD 接口（J8）只有 4 根线（SWDIO、SWCLK、GND、3.3V），**没有 nRESET 引脚**
- **解决**：改为 `reset_config none`，使用软件复位（SYSRESETREQ）通过 SWD 调试接口复位内核
- **正确 `openocd.cfg` 配置**：
  ```cfg
  adapter driver cmsis-dap
  cmsis_dap_backend auto
  transport select swd
  cmsis_dap_vid_pid 0xfaed 0x4870
  source [find target/stm32f4x.cfg]
  adapter speed 500
  reset_config none
  ```
- **教训**：`reset_config` 必须与硬件复位引脚连接情况匹配：
  - `srst_only`：需要 SRST 引脚 → C 板 SWD 不支持
  - `none`：软件复位 → 不需要额外引脚，C 板可用
- **烧录命令**：
  ```bash
  make flash
  # 等效于: openocd -f openocd.cfg -c "program build/demo1_imu.elf verify reset exit"
  ```

### CMSIS-DAP 连接问题排查
- **设备识别**：`lsusb | grep faed` → 应看到 `ID faed:4870 Horco CMSIS-DAP`
- **权限**：用户需在 `plugdev` 组，否则 `sudo` 或添加 udev 规则
- **后端**：`cmsis_dap_backend auto` 自动选择最佳后端（Linux 上用 usb_bulk）
- **固件版本**：`FW Version = Horco v0.2`（C 板内置 DAPLink）

---

## 一、IMU / BMI088 错误

### 1. SPI 数据解析右移问题
- **现象**：加速度、角速度数据异常，数值不对
- **原因**：BMI088 寄存器数据是 16 位，高低字节拼接后需要右移处理
- **解决**：数据拼接时正确移位

### 2. SPI 配置错误
- **现象**：BMI088 初始化失败，读不到芯片 ID
- **原因**：SPI 时钟极性/相位配置不匹配 BMI088 要求
- **正确配置**：`CPOL=HIGH, CPHA=2EDGE`（模式3）

### 3. 初始化时序
- **现象**：偶尔初始化失败
- **原因**：BMI088 上电后需要延时才能稳定响应
- **解决**：加 `HAL_Delay` 等待传感器就绪

---

## 二、CAN / GM6020 电机错误（共7个Bug）

### Bug 1: CAN 波特率偏10% ⭐⭐
- **现象**：CAN 发送成功但无应答，LEC=5（位支配错误）
- **原因**：Prescaler=2, BS1=13, BS2=5 → 实际波特率 1.105MHz，偏离 1Mbps 约 10%
- **正确配置**：
  ```
  Prescaler = 3, BS1 = 10TQ, BS2 = 3TQ
  42MHz / 3 / (1+10+3) = 1.0MHz ✓
  ```
- **教训**：CAN 波特率必须精确匹配，10% 偏差也会导致通信失败

### Bug 2: AutoRetransmission 导致邮箱卡死 ⭐⭐
- **现象**：发送失败后 CAN 发送函数不再返回 HAL_OK，程序卡住
- **原因**：`AutoRetransmission=ENABLE` 时，如果总线无应答，CAN 控制器会一直重试，占满 3 个邮箱，后续发送全部失败
- **解决**：改为 `AutoRetransmission=DISABLE`，发送一次不重试，邮箱不会卡死

### Bug 3: CANH / CANL 接反 ⭐⭐
- **现象**：LEC=5（Bit Dominant Error），CAN 总线电平异常
- **原因**：C 板的 CANH 接到了电机的 CANL，CANL 接到了 CANH
- **解决**：对调 CANH 和 CANL 接线
- **教训**：CAN 是差分信号，CAN_H 接 CAN_H，CAN_L 接 CAN_L

### Bug 4: 终端电阻未开启 ⭐⭐
- **现象**：LEC=5（Bit Dominant Error），CAN 总线电平异常
- **原因**：GM6020 拨码开关 Bit4（CAN Resistor）未拨到 ON，总线缺少 120Ω 终端电阻
- **解决**：将 GM6020 拨码开关 Bit4 拨到 ON，开启 120Ω 终端电阻
- **教训**：CAN 总线两端都必须有终端电阻（至少一端），否则信号反射导致通信失败

### Bug 5: Bus-Off 无自动恢复
- **现象**：接线错误后 TEC=128, REC=255，CAN 控制器进入 Bus-Off 状态，无法恢复
- **原因**：错误计数器累积到高位后，即使物理连接修复，控制器也不会自动清零
- **解决**：检测到 TEC>=128 或 REC>=128 时，主动调用：
  ```c
  HAL_CAN_ResetError(&hcan1);
  HAL_CAN_Stop(&hcan1);
  HAL_CAN_Start(&hcan1);
  ```

### Bug 6: CAN 中断未触发
- **现象**：HAL_CAN_RxFifo0MsgPendingCallback 不触发，收不到电机反馈
- **原因**：CAN RX 中断未使能
- **解决**：
  ```c
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  ```

### Bug 7: 控制报文ID错一位 + 数组索引混乱 ⭐⭐⭐（最终根因）
- **现象**：CAN 通信正常（recv_all/recv_cnt 都涨），但 motor[4].online=0, ecd=0, output=0，电机不转
- **原因链**（三个问题叠加）：

#### 7a. 控制ID写错
```
错误：0x2FE → 正确：0x2FF（ID5-7控制报文）
      0x1FE → 正确：0x1FF（ID1-4控制报文）
```
参考 RM2026 官方框架确认：控制报文 ID 是 0x1FF/0x2FF，不是 0x1FE/0x2FE

#### 7b. 反馈索引多减1
```c
// 错误：多减了1，导致 ID5(0x209) 存到 gimbal_motor[4] 而不是 [5]
uint8_t id = rx_header.StdId - GM6020_FEEDBACK_ID_BASE - 1;

// 正确：直接用差值作为索引
uint8_t id = rx_header.StdId - GM6020_FEEDBACK_ID_BASE;
```
GM6020 反馈 ID 映射：
| 电机ID | 反馈报文ID | 计算得id | 应存入 |
|--------|-----------|---------|--------|
| 1 | 0x205 | 1 | motor[1] |
| 5 | 0x209 | 5 | motor[5] |

#### 7c. set_target 和 update 索引不一致
```c
// 错误：set_target 用 id-1，update 读 [4]，反馈存到 [5]
gimbal_target[id - 1] = angle;   // target[4]
gimbal_output[4];                 // 读 output[4]
gimbal_motor[5].angle_deg;        // 但反馈在 motor[5]

// 正确：统一用物理ID当数组索引
gimbal_target[id] = angle;        // target[5]
gimbal_output[5];                 // 读 output[5]
gimbal_motor[5].angle_deg;        // 反馈也在 motor[5] ✓
```

**最终效果**：PID 计算 `target[5] - motor[5].angle_deg` → 输出 `output[5]` → 发送到 0x2FF 报文 ✅

---

## 三、AHRS / 姿态解算错误

### Bug 8: Yaw 轴持续漂移（顺时针旋转） ⭐⭐
- **现象**：云台静止时 yaw 角持续顺时针漂移
- **原因**：BMI088 无磁力计，`twoKi=0` 零偏未补偿
- **解决**：上电 1 秒静止校准 + 启用积分项 `twoKi=0.01` + `ahrs_update` 减去零偏
- **关键代码**：[ahrs.c](file:///home/sz/coderepo/robomater_game/gimbal/modules/imu/ahrs/ahrs.c#L39-L44)

### Bug 9: Yaw 轴电机有时不出力 ⭐⭐
- **现象**：yaw 电机有时不推，误差一直存在不锁定
- **原因链**（两个问题叠加）：

#### 9a. 角度环 Ki=0
```c
// 旧：纯比例控制，没有积分消除稳态误差
pid_init(&gimbal_pid[5], 15.0f, 0.0f, 0.0f, ...);
```

#### 9b. 积分衰减太激进
```c
// 旧：abs_error < 1° 时每周期衰减 2%
// 200Hz 下 1 秒后积分仅剩 1.8%，需要持续出力时积分瞬间没了
if (abs_error < 1.0f) {
    pid->integral *= 0.98f;
}
```

- **解决**：
  1. yaw 角度环 Ki = 0.05（[gimbal.c](file:///home/sz/coderepo/robomater_game/gimbal/modules/gimbal/motor/gimbal.c#L124)）
  2. 衰减调缓到 0.999（[pid.c](file:///home/sz/coderepo/robomater_game/gimbal/modules/gimbal/pid/pid.c#L43)）

- **效果**：积分能累积并保持，yaw 持续出力顶住误差

---

## 四、调试方法论

### 逐级排查 CAN 通信
1. **物理层**：接线（CANH/CANL）、终端电阻、供电（24V）
2. **波特率**：精确匹配 1Mbps
3. **CAN 控制器**：检查 ESR 寄存器（TEC、REC、LEC）
4. **中断**：确认 RX 中断使能、回调函数触发
5. **数据**：校验接收到的数据是否正确

### VOFA+ 调试技巧
- 把 CAN 调试信息（send_cnt、recv_cnt、TEC、LEC、ESR）发送到 VOFA+
- 通过 LEC 快速定位错误类型：
  - `LEC=0` 无错误
  - `LEC=3` ACK 错误（无人应答）
  - `LEC=5` 位支配错误（接线/终端电阻问题）
  - `LEC=2` 格式错误（波特率不匹配）

### 抓取实际 CAN ID 定位索引问题
- 当 recv_all == recv_cnt 但 motor 数据全为 0 时，说明所有消息都通过了 ID 匹配，但存错了位置
- 在回调中记录 `rx_header.StdId` 到 VOFA+，对比实际收到的 ID 和预期 ID
- 本例中实际收到 0x209，确认是 ID5 的反馈，应该存到 motor[5]

---

## 五、GM6020 协议速查

### 报文 ID 定义
| 功能 | ID范围 | 说明 |
|------|--------|------|
| 控制 ID1-4 | 0x1FF | 电流值填入 data[0~7] |
| 控制 ID5-7 | 0x2FF | 电流值填入 data[0~7] |
| 反馈 ID1-4 | 0x201~0x204 | M3508/M2006 用 |
| **反馈 ID5-7** | **0x205~0x20B** | **GM6020 用！注意起始地址不同** |

### 反馈数据格式（8字节）
| 字节 | 含义 |
|------|------|
| [0]-[1] | 编码器值 ecd (0~8191), 高字节在前 |
| [2]-[3] | 速度 rpm, 有符号16位 |
| [4]-[5] | 电流 mA, 有符号16位 |
| [6] | 温度 °C |
| [7] | 预留 |

### 控制数据格式（8字节，ID5-7用0x2FF）
| 字节 | 含义 |
|------|------|
| [0]-[1] | 电机1电流 (int16) |
| [2]-[3] | 电机2电流 (int16) |
| [4]-[5] | 电机3电流 (int16) |
| [6]-[7] | 电机4电流 (int16) |

### 角度计算
```c
angle_deg = ecd * 360.0f / 8192.0f;
```

---

## 六、文件结构

```
demol/
├── Core/Src/main.c              # 主程序：IMU+电机+VOFA+
├── modules/
│   ├── imu/
│   │   ├── bmi088/bsp_bmi088.c  # BMI088 SPI驱动
│   │   ├── ahrs/ahrs.c          # Mahony姿态解算
│   │   └── vofa/bsp_vofa.c      # VOFA+ JustFloat协议
│   └── gimbal/
│       ├── pid/pid.c            # PID控制器
│       └── motor/
│           ├── gimbal.h         # GM6020数据结构和接口
│           └── gimbal.c         # CAN通信+PID控制逻辑
├── DEBUG_LOG.md                 # 本文档
└── vofa_fields.csv              # VOFA+字段表
```
# 电控组第一部分考核

用 C 板（STM32F407IGH6）通过 CAN 控制 GM6020 电机，跟随正弦速度曲线与正弦位置曲线。
曲线参数可自定义，并启用看门狗保护。

## 实现

| 项目 | 说明 |
|---|---|
| 通信 | CAN1，1 Mbps，标准帧 |
| 控制周期 | 1 kHz（TIM6 中断） |
| 速度模式 | 目标转速 − 实际转速 → PID → 控制量（单环） |
| 位置模式 | 目标角度 − 实际角度 → 位置 PID → 目标转速 → 速度 PID → 控制量（双环串级） |
| 看门狗 | IWDG 0.8 s，在 1 kHz 控制循环里喂狗 |
| 其他保护 | 反馈超时 100 ms、过温 70 ℃、输出限幅 |

## 目录结构

```
Core/                      CubeMX 生成的初始化代码
  Src/control_task_hooks.c   HAL 回调接到控制任务
Tasks/                     业务代码
  inc/  gm6020.hpp  pid.hpp  trajectory.hpp  control_task.hpp
  src/  对应实现
_test/                     PC 端测试（协议 / PID / 轨迹，与硬件无关）
C_Board_Assessment.jdebug  Ozone 工程，打开后可在 Timeline 看波形
```

`Tasks/inc/control_task.hpp` 对外接口与官方参考的 `ControlTask.h` 一致：

```c
extern "C" {
void ControlTaskInit(void);
void MainTask(void);
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);
}
```

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/C_Board_Assessment.elf`、`build/Debug/C_Board_Assessment.hex` / `.bin`

## 常用改动位置

```c
// Tasks/src/trajectory.cpp —— 曲线参数
kVelAmp  6.0f     速度正弦幅值 rad/s
kVelFreq 0.25f    速度正弦频率 Hz
kPosAmp  2.0f     位置正弦幅值 rad
kPosFreq 0.20f    位置正弦频率 Hz

// Tasks/src/control_task.cpp —— 跟随哪条曲线
kDefaultMode = CTRL_MODE_VEL 或 CTRL_MODE_POS

// Tasks/src/control_task.cpp —— PID 参数
kVelKp / kVelKi / kPosKp

// Tasks/inc/gm6020.hpp —— 控制协议
GM6020_USE_CURRENT_MODE  0 = 电压控制(0x1FF)，1 = 电流控制(0x1FE)
```

## 实测结果

| | 目标 | 跟踪误差 RMS |
|---|---|---|
| 速度模式 | 6.0·sin(2π·0.25·t) rad/s | 0.35 rad/s（5.8%） |
| 位置模式 | 2.0·sin(2π·0.20·t) rad | 9.9°（8.6%） |

波形截图取自 Ozone 的 Timeline（数据采样 1 kHz），红为目标、绿为实测。

## 注意

- 下载前确认 CubeMX 里 `System Core → SYS → Debug` 选的是 `Serial Wire`，否则会丢失调试口。
- 电机若处于电流控制模式（指示灯橙灯常亮），需把 `GM6020_USE_CURRENT_MODE` 设为 `1`；
  电压模式（绿灯闪）则设为 `0`。两种模式的 PID 参数不同，不能混用。
- Ozone 连接时会暂停 CPU，此时电机会因收不到 ACK 而掉线。应先按 `F5` 让程序运行，
  再给电机上电。

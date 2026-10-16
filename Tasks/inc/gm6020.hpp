/**
*******************************************************************************
* @file  : gm6020.hpp
* @brief : GM6020 直流无刷电机 CAN 协议驱动
*          协议依据：官方资料 C_board_src/Gm6020.h/.cpp
*******************************************************************************
*/

#ifndef GM6020_HPP
#define GM6020_HPP

#include <stdint.h>

/* 控制模式：1 = 电流控制（0x1FE/0x2FE，给定 ±16384 对应 ±3A），
 * 0 = 电压控制（0x1FF/0x2FF，给定 ±25000）。本工程电机为电流模式。 */
#define GM6020_USE_CURRENT_MODE  (1)

/* 控制量满量程绝对值，用于 PID 输出限幅：电压模式 ±25000，电流模式 ±16384 */
#if GM6020_USE_CURRENT_MODE
constexpr int16_t kGm6020CmdMax = 16384;
#else
constexpr int16_t kGm6020CmdMax = 25000;
#endif

class Gm6020
{
public:
  /** @brief 构造 @param id 电机拨码开关设置的 ID（1~7） */
  explicit Gm6020(uint8_t id);

  uint32_t RxId() const;   /* 反馈帧 ID：0x204 + id */
  uint32_t TxId() const;   /* 控制帧 ID：0x1FF/0x1FE（id<=4）、0x2FF/0x2FE（id>=5） */

  float   Angle() const;    /* 累计机械角度，单位 rad，支持多圈 */
  float   Vel() const;      /* 转速，单位 rad/s */
  float   Current() const;  /* 转矩电流，单位 A */
  float   Temp() const;     /* 温度，单位 摄氏度 */
  int16_t VelRpm() const;   /* 原始转速，单位 rpm */

  /* 函数名沿用官方实现；电压模式为电压给定 ±25000，电流模式为电流给定 ±16384（±3A） */
  void    SetVoltage(int16_t v);   /* 控制量给定，满量程见 kGm6020CmdMax */
  int16_t Voltage() const;         /* 取回当前控制量给定（调试用）*/

  void Decode(const uint8_t *data);        /* 解析 8 字节反馈帧 */
  void Encode(uint8_t *data) const;        /* 把本电机控制量写入控制帧对应位置 */

private:
  uint8_t id_;               /* 电机 ID */
  int16_t voltage_;          /* 本拍控制量给定（电压或电流） */

  uint16_t raw_angle_;       /* 圈内机械角度，范围 0~8191 */
  int16_t  raw_vel_rpm_;     /* 转速原始值，单位 rpm */
  int16_t  raw_current_;     /* 转矩电流原始值 */
  uint8_t  raw_temp_;        /* 温度，摄氏度 */

  int32_t  full_turns_;      /* 累计整圈数：正转 +1，反转 -1 */
  uint16_t last_raw_angle_;  /* 上一拍原始角度，用于跨零检测 */
};

#endif /* GM6020_HPP */

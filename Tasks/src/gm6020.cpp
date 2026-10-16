/**
*******************************************************************************
* @file  : gm6020.cpp
* @brief : GM6020 CAN 协议驱动实现
*          协议依据：官方资料 C_board_src/Gm6020.cpp
*******************************************************************************
*/

#include "gm6020.hpp"

namespace
{
const float    kPi         = 3.14159265358979323846f;

/* 一圈编码器刻度数 0~8191，共 8192 */
const uint16_t kAngleRange = 8192;

/* rad/s = rpm * 2π / 60 */
const float    kRpmToRadS  = (2.0f * kPi) / 60.0f;

/* 满量程 ±16384 约对应 ±3 A，1 刻度 = 3/16384 A */
const float    kRawToAmp   = 3.0f / 16384.0f;

/* 跨零判据阈值：半圈 = 8192 / 2 */
const int16_t kHalfTurn = 4096;
}

/** @brief 构造，全部成员清零 */
Gm6020::Gm6020(uint8_t id)
  : id_(id), voltage_(0),
    raw_angle_(0), raw_vel_rpm_(0), raw_current_(0), raw_temp_(0),
    full_turns_(0), last_raw_angle_(0)
{
}

/** @brief 反馈帧标识符：0x204 + id */
uint32_t Gm6020::RxId() const
{
  return 0x204u + id_;
}

/* 控制帧标识符：ID 1~4 用 0x1FF（电流模式 0x1FE），ID 5~7 用 0x2FF（0x2FE） */
uint32_t Gm6020::TxId() const
{
#if GM6020_USE_CURRENT_MODE
  return (id_ <= 4) ? 0x1FEu : 0x2FEu;
#else
  return (id_ <= 4) ? 0x1FFu : 0x2FFu;
#endif
}

/** @brief 解析 8 字节反馈帧 @param data CAN 原始数据（16 位量为大端序）
 *  data[0..1] 机械角度 0~8191，[2..3] 转速 rpm，[4..5] 转矩电流，[6] 温度，[7] 保留 */
void Gm6020::Decode(const uint8_t *data)
{
  uint16_t raw = (uint16_t)((data[0] << 8) | data[1]);

  /* 跨零检测：1 ms 内角度不可能真变化半圈，跳变超过 4096 说明越过了 0 点 */
  int16_t diff = (int16_t)(raw - last_raw_angle_);

  if (diff > kHalfTurn)
  {
    full_turns_--;             /* 反向跨零（角度递减越过 0 点），圈数 -1 */
  }
  else if (diff < -kHalfTurn)
  {
    full_turns_++;             /* 正向跨零（角度递增越过 0 点），圈数 +1 */
  }

  last_raw_angle_ = raw;       /* 记录本拍角度，下一拍用于比较 */

  raw_angle_   = raw;
  raw_vel_rpm_ = (int16_t)((data[2] << 8) | data[3]);
  raw_current_ = (int16_t)((data[4] << 8) | data[5]);
  raw_temp_    = data[6];
}

/** @brief 累计机械角度，单位 rad（连续多圈） */
float Gm6020::Angle() const
{
  return ((float)full_turns_ * kAngleRange + raw_angle_)
         * (2.0f * kPi / kAngleRange);
}

/** @brief 转速，单位 rad/s */
float Gm6020::Vel() const
{
  return raw_vel_rpm_ * kRpmToRadS;
}

/** @brief 转矩电流，单位 A（近似值） */
float Gm6020::Current() const
{
  return raw_current_ * kRawToAmp;
}

/** @brief 电机温度，单位 摄氏度 */
float Gm6020::Temp() const
{
  return (float)raw_temp_;
}

/** @brief 原始转速，单位 rpm */
int16_t Gm6020::VelRpm() const
{
  return raw_vel_rpm_;
}

/** @brief 设定本拍控制量 @param v 电压模式 ±25000，电流模式 ±16384（对应 ±3A） */
void Gm6020::SetVoltage(int16_t v)
{
  voltage_ = v;
}

/** @brief 取回当前控制量给定（调试用） */
int16_t Gm6020::Voltage() const
{
  return voltage_;
}

/** @brief 把本拍控制量写入 8 字节控制帧 @param data 8 字节数组，由调用者初始化并发送
 *  ID 1~4 依次占 [0,1]~[6,7]，ID 5~7 在第二组内从 [0,1] 起，每个电机 2 字节（大端） */
void Gm6020::Encode(uint8_t *data) const
{
  /* 起始下标 = (组内序号 - 1) × 2，ID 5~7 需先减 4 */
  uint8_t index = (id_ <= 4) ? (uint8_t)((id_ - 1) * 2)
                             : (uint8_t)((id_ - 5) * 2);

  data[index]     = (uint8_t)((voltage_ >> 8) & 0xFF);
  data[index + 1] = (uint8_t)(voltage_ & 0xFF);
}

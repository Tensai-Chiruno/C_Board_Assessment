/**
*******************************************************************************
* @file  : pid.cpp
* @brief : 位置式 PID 控制器实现
* 用于速度环（目标转速 - 实际转速 -> 电压）与位置环（目标角度 - 实际角度 -> 目标转速）
*******************************************************************************
*/

#include "pid.hpp"

namespace
{
/* 控制周期：TIM6 中断 1 kHz，即 0.001 s */
const float kSampleTime = 0.001f;
}

/** @brief 构造函数：设定 PID 增益与输出上下限 */
Pid::Pid(float kp, float ki, float kd, float out_max, float out_min)
  : kp_(kp), ki_(ki), kd_(kd),
    out_max_(out_max), out_min_(out_min),
    integ_(0.0f), last_err_(0.0f)
{
}

/** @brief 修改 PID 三个增益（运行时调整用） */
void Pid::SetParams(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

/** @brief 修改输出上下限 */
void Pid::SetOutputLimit(float out_max, float out_min)
{
  out_max_ = out_max;
  out_min_ = out_min;
}

/** @brief 复位控制器内部状态（清空积分累加值与上一拍误差） */
void Pid::Reset()
{
  integ_    = 0.0f;
  last_err_ = 0.0f;
}

// 位置式 PID：u(k) = Kp*e(k) + Ki*sum(e(j))*T + Kd*(e(k)-e(k-1))/T
// 输出饱和时撤销本拍积分累加（积分回退抗饱和）
float Pid::Calc(float ref, float fdb)
{
  /* 本拍误差 = 目标 - 实际 */
  float err   = ref - fdb;

  /* 误差差分（不是反馈值差分），供微分项使用 */
  float deriv = err - last_err_;

  /* 累加本拍误差；只存 sum(e(j))，Ki 与 T 在下面一起乘 */
  integ_ += err;

  /* 三项相加，得到未限幅的输出 */
  float out = kp_ * err
            + ki_ * integ_ * kSampleTime
            + kd_ * deriv / kSampleTime;

  /* 输出限幅：饱和时撤销本拍积分累加，抑制积分饱和 */
  if (out > out_max_)
  {
    out = out_max_;
    integ_ -= err;   /* 撤销本拍的积分累加 */
  }
  else if (out < out_min_)
  {
    out = out_min_;
    integ_ -= err;   /* 撤销本拍的积分累加 */
  }

  /* 保存本拍误差，下一拍算微分时要用 */
  last_err_ = err;

  return out;
}

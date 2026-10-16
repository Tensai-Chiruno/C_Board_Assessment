/**
*******************************************************************************
* @file  : trajectory.cpp
* @brief : 正弦轨迹生成实现，为 PID 提供速度/角度目标值
* 速度曲线： v(t) = kVelAmp * sin(2*pi*kVelFreq*t)
* 位置曲线： th(t) = kPosBias + kPosAmp * sin(2*pi*kPosFreq*t)
* t = 周期计数 × 0.001 s，与 1 kHz 控制循环同步。
*******************************************************************************
*/

#include "trajectory.hpp"

/* ==================== 【功能】考核要求的自定义曲线参数 ==================== */

#define kVelAmp           (6.0f)     /* 速度正弦幅值  rad/s */
#define kVelFreq          (0.25f)    /* 速度正弦频率  Hz    */

#define kPosAmp           (2.0f)     /* 位置正弦幅值  rad，不超过 ±pi */
#define kPosFreq          (0.20f)    /* 位置正弦频率  Hz    */
#define kPosBias          (0.0f)     /* 位置偏置      rad   */

/* 控制周期 1 kHz，即每拍 1 ms */
#define kCtrlPeriod       (0.001f)

/* ================================================================= */

namespace
{
const float kPi  = 3.14159265358979323846f;
const float k2Pi = 2.0f * kPi;

/* 【功能】折算到 [-pi, pi]：sin 以 2*pi 为周期，且下面的多项式近似只在该区间准确 */
float WrapPi(float x)
{
  while (x > kPi)
  {
    x -= k2Pi;
  }
  while (x < -kPi)
  {
    x += k2Pi;
  }
  return x;
}

// 【功能】sin 的 7 阶泰勒近似，输入需已折算到 [-pi, pi]；比标准库 sin() 开销小
float SinApprox(float x)
{
  float x2 = x * x;
  return x * (1.0f
              - x2 / 6.0f
              + x2 * x2 / 120.0f
              - x2 * x2 * x2 / 5040.0f);
}

/** @brief 【功能】当前相位 2*pi*f*t
 *  @param tick 已走过的控制周期数
 *  @param freq 正弦频率，单位 Hz */
float Phase(uint32_t tick, float freq)
{
  return k2Pi * freq * ((float)tick * kCtrlPeriod);
}
}

/* 【功能】构造：周期计数从 0 开始，即 t = 0 */
Trajectory::Trajectory() : tick_(0)
{
}

/* 【调试】从 0 时刻重新开始，只有运行时切换模式才会调用 */
void Trajectory::Reset()
{
  tick_ = 0;
}

/* 【功能】推进一个控制周期，1 kHz 控制中断里每拍调用一次 */
void Trajectory::Step()
{
  tick_++;
}

/* 【调试】当前运行时间，单位 s，只有 PC 测试程序用 */
float Trajectory::Time() const
{
  return (float)tick_ * kCtrlPeriod;
}

/* 【功能】本拍目标转速，单位 rad/s，作为速度环目标 */
float Trajectory::VelRef() const
{
  return kVelAmp * SinApprox(WrapPi(Phase(tick_, kVelFreq)));
}

/* 【功能】本拍目标角度，单位 rad；比速度曲线多一个 kPosBias 偏置 */
float Trajectory::PosRef() const
{
  return kPosBias + kPosAmp * SinApprox(WrapPi(Phase(tick_, kPosFreq)));
}

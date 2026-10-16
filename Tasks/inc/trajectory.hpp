/**
 * @file  : trajectory.hpp
 * @brief : 正弦轨迹生成：v(t) = Vm*sin(2*pi*f*t)，th(t) = th0 + Am*sin(2*pi*f*t)，参数可自定义 */

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <stdint.h>

class Trajectory
{
public:
  Trajectory();

  /* 【调试】从 0 时刻重新开始，只有运行时切换模式才会调用 */
  void Reset();

  /* 【功能】每个控制周期调用一次，推进内部时间 */
  void Step();

  /* 【功能】目标速度，单位 rad/s */
  float VelRef() const;

  /* 【功能】目标角度，单位 rad */
  float PosRef() const;

  /* 【调试】当前运行时间，单位 s，只有 PC 测试程序用 */
  float Time() const;

private:
  uint32_t tick_;   /* 已走过的控制周期数 */
};

#endif /* TRAJECTORY_HPP */

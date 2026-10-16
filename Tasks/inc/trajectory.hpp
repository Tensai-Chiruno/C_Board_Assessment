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

  /* 从 0 时刻重新开始 */
  void Reset();

  /* 每个控制周期调用一次，推进内部时间 */
  void Step();

  /* 目标速度，单位 rad/s */
  float VelRef() const;

  /* 目标角度，单位 rad */
  float PosRef() const;

  /* 当前运行时间，单位 s */
  float Time() const;

private:
  uint32_t tick_;   /* 已走过的控制周期数 */
};

#endif /* TRAJECTORY_HPP */

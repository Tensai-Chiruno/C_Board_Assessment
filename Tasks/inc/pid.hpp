/**
*******************************************************************************
* @file  : pid.hpp
* @brief : 位置式 PID 控制器（含输出限幅与积分回退抗饱和）
* 离散式：u(k) = Kp*e(k) + Ki*sum(e(j))*T + Kd*(e(k)-e(k-1))/T
*******************************************************************************
*/

#ifndef PID_HPP
#define PID_HPP

class Pid
{
public:
  Pid(float kp, float ki, float kd, float out_max, float out_min);

  void SetParams(float kp, float ki, float kd);
  void SetOutputLimit(float out_max, float out_min);
  void Reset();

  /** @brief 计算一次控制量（ref 目标值、fdb 反馈值），返回限幅后的输出 */
  float Calc(float ref, float fdb);

  float Kp() const { return kp_; }
  float Ki() const { return ki_; }
  float Kd() const { return kd_; }

private:
  float kp_, ki_, kd_;
  float out_max_, out_min_;
  float integ_;     /* 误差累积 sum(e(j)) */
  float last_err_;  /* 上一拍误差 e(k-1) */
};

#endif /* PID_HPP */

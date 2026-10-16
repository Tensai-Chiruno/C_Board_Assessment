/**
 * @file  : control_task.cpp
 * @brief : 控制主任务实现（1 kHz 控制循环 + CAN 收发 + 看门狗）
 *        MainTask() 每 1 ms 由 TIM6 中断调用：喂狗 -> 推进轨迹 -> 安全检查 -> PID -> 发帧。
 *        CanFeedbackCallback() 在 CAN 接收中断里解析反馈帧，更新电机对象。
 *
 * 【代码标注约定】本工程各文件统一用下面两个标记区分代码性质：
 *   【功能】 实现考核要求的代码 —— 控制、通信、安全保护。删掉程序就不工作。
 *   【调试】 调试/诊断用的代码 —— 计数器、波形输出、备用调参接口。
 *            删掉后控制功能完全不变，只影响观察和排障。
 */

#include "control_task.hpp"

#include "gm6020.hpp"
#include "pid.hpp"
#include "trajectory.hpp"

/* CubeMX 生成的外设头文件，声明了 hcan1 / htim6 / hiwdg 等句柄 */
#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"

/* ============================ 可调参数 ============================ */

/* 电机 ID：必须和电机上拨码开关设的一致 */
#define kMotorId            (1u)

/* 默认控制模式：CTRL_MODE_VEL 跟踪速度正弦，CTRL_MODE_POS 跟踪位置正弦 */
#define kDefaultMode        (CTRL_MODE_VEL)

/* 控制量限幅，远小于协议满量程（电压 ±25000 / 电流 ±16384），首次上电整定时留安全余量 */
#define kCmdMax             (3000.0f)
#define kCmdMin             (-3000.0f)

/* 速度环参数：输入误差单位 rad/s，输出是控制量给定，故 Kp 量纲为 控制量/(rad/s)。
 * GM6020_USE_CURRENT_MODE 决定控制量走电压还是电流协议，两套参数不能互换——
 * 被控对象不同：电压模式近似一阶惯性、直流增益小，Kp 要大；电流模式驱动器内部已做
 * 转矩电流闭环，控制量->转矩->角加速度->转速是纯积分，增益高三个数量级，Kp 必须小得多。 */
#if GM6020_USE_CURRENT_MODE
#define kVelKp              (250.0f)
#define kVelKi              (2000.0f)
#else
#define kVelKp              (1500.0f)
#define kVelKi              (200.0f)
#endif
#define kVelKd              (0.0f)      /* 微分：先不加，需要抑制超调时再加 */

/* 【调试】实际转速由角度差分得到，不用电机自报值：自报值有低频误差，而差分有半个窗口的滞后，
 * 会吃掉高增益速度环的相位裕度，所以差分结果只用于调试输出 dbgActual，不进控制回路。
 * 窗口取 20 拍（20 ms），把 1 ms 内几个计数变化的量化误差摊薄。 */
#define kVelWinTicks        (20u)
#define kVelWinSeconds      (0.001f * (float)kVelWinTicks)

/* 摩擦前馈参数（只在电流模式下有意义；电压模式下摩擦靠转矩电压自然克服，故置 0 关掉） */
#if GM6020_USE_CURRENT_MODE
#define kFricFf             (430.0f)    /* 摩擦补偿量，单位同控制量（电流计数） */
#define kFricFfSoft         (1.0f)      /* 软化尺度，单位 rad/s，越小越接近方波 */
#else
#define kFricFf             (0.0f)
#define kFricFfSoft         (1.0f)
#endif

/* 位置环（串级外环）参数：输入误差单位 rad，输出目标转速 rad/s，Kp 量纲 1/s，即外环带宽。
 * 只加 P 不加 I：被控对象（目标转速->角度）本身就是积分环节，单 P 已能消除常值位置误差。
 * 电流模式内环带宽高，Kp 给 10；电压模式内环慢，保守给 2.0。 */
#if GM6020_USE_CURRENT_MODE
#define kPosKp              (10.0f)
#else
#define kPosKp              (2.0f)
#endif
#define kPosKi              (0.0f)
#define kPosKd              (0.0f)

/* 外环输出限幅，同时限制了电机的最大目标转速 */
#define kPosOutMax          (20.0f)
#define kPosOutMin          (-20.0f)

/* 反馈超时拍数：1 kHz 下 100 拍 = 100 ms，连续收不到 CAN 反馈就停机 */
#define kFeedbackTimeout    (100u)

/* 电机温度上限，超过就停机保护 */
#define kTempLimit          (70.0f)

/* ===== CAN 总线异常保护 =====
 * C 板与电机无公共地，电机吃电流时共模漂移产生位错误：C 板 TEC 涨到 256 先进总线关闭
 * -> 不再给电机的反馈帧回 ACK -> 电机也跟着锁死（电机侧 ABOM 关闭，进去须断电重启）。
 * 措施：can.c 关掉自动重传（one-shot），加上这里邮箱占满时主动中止。
 * 不能改成"TEC 超阈值就停发"：TEC 只能靠成功发送才下降，停发会死锁。 */

/* ================================================================= */

/* 【调试】调试器采样用的全局量：dbgTarget / dbgActual 供 Ozone 采样画波形，不参与控制计算。
 * 必须定义在匿名命名空间之外——内部链接的符号调试器按名字找不到；volatile 防优化。
 * 单位：速度模式 rad/s，位置模式 rad。 */
volatile float dbgTarget = 0.0f;
volatile float dbgActual = 0.0f;

/* ================================================================= */

/* 匿名命名空间：本文件私有的全局对象和函数 */
namespace
{
Gm6020     motor(kMotorId);
Trajectory traj;

/* 速度环输出限幅用控制量范围，位置环输出限幅用转速范围 */
Pid velPid(kVelKp, kVelKi, kVelKd, kCmdMax, kCmdMin);
Pid posPid(kPosKp, kPosKi, kPosKd, kPosOutMax, kPosOutMin);

uint32_t mode      = kDefaultMode;  /* 当前控制模式 */
uint32_t timeout   = 0;             /* 反馈超时计数，收到反馈就清零 */
uint8_t  txBuf[8];                  /* CAN 控制帧的 8 字节缓冲区 */

/* 【调试】诊断计数器：这个数一直涨说明总线上反复出错、帧发不出去 */
uint32_t txAbortN = 0;              /* 累计主动中止发送邮箱的次数 */

/* 【调试】角度差分窗口的状态（只服务于 dbgActual 波形，不进控制回路） */
float velAngHist[kVelWinTicks + 1u];   /* 角度环形缓冲，用于滑动窗口差分 */
uint32_t velAngIdx  = 0;               /* 环形缓冲写指针 */
uint32_t velAngFill = 0;               /* 已写入的样本数，填满后才出有效值 */

/** @brief 【功能】把当前控制量给定组帧并通过 CAN1 发出去（发不出去就跳过本拍，1 ms 后会再发） */
void SendControlFrame()
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t mailbox;   /* 发送成功后 HAL 会把占用的邮箱编号写到这里 */

  /* 三个发送邮箱都占着说明前面的帧一直没人 ACK，主动中止，否则新指令永远轮不上；
   * 只中止、不停发——停发会让 TEC 再也降不下来。 */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
    (void)HAL_CAN_AbortTxRequest(&hcan1,
                                 CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    txAbortN++;
    return;
  }

  /* 先清零，避免带着上一帧的残留数据发给同一标识符下的其他电机 */
  for (uint8_t i = 0; i < 8; i++)
  {
    txBuf[i] = 0;
  }

  /* 让电机对象把自己的控制量写进缓冲区里对应的两个字节 */
  motor.Encode(txBuf);

  txHeader.StdId              = motor.TxId();      /* 标准帧 ID：0x1FF 或 0x2FF */
  txHeader.ExtId              = 0;
  txHeader.IDE                = CAN_ID_STD;
  txHeader.RTR                = CAN_RTR_DATA;
  txHeader.DLC                = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  (void)HAL_CAN_AddTxMessage(&hcan1, &txHeader, txBuf, &mailbox);
}

/** @brief 【功能】停机：控制量归零，并清掉 PID 积分（否则恢复时会带着停机前的累积值猛冲一下） */
void Stop()
{
  motor.SetVoltage(0);
  velPid.Reset();
  posPid.Reset();

  /* 角度差分窗口也清掉：反馈断开期间窗口里还是断线前的老角度，一恢复会算出巨大假速度 */
  velAngFill = 0;

  SendControlFrame();   /* 把 0 控制量发给电机，让它松力 */
}

/**
 * @brief 【功能】摩擦前馈：按运动方向补一个恒定分量
 * @param velCmd 本拍希望电机达到的转速，单位 rad/s
 * @return 要叠加到 PID 输出上的补偿量
 *         静摩擦约合 430 个电流计数，目标速度过零时 PID 输出接近 0 推不动，跟踪会一顿一顿；
 *         用线性饱和替代 sign()，过零附近平滑过渡，避免 1 kHz 下电流抖动。
 */
float FrictionFeedforward(float velCmd)
{
  float s = velCmd / kFricFfSoft;

  if (s > 1.0f)
  {
    s = 1.0f;
  }
  else if (s < -1.0f)
  {
    s = -1.0f;
  }

  return kFricFf * s;
}

/**
 * @brief 【调试】用角度差分算实际转速（滑动窗口），结果只给 dbgActual 波形看，不进控制回路
 * @param angleNow 本拍的累计角度，单位 rad
 * @return 窗口两端之间的平均角速度，单位 rad/s；缓冲未填满时返回 0
 */
float AngleVelocity(float angleNow)
{
  float v = 0.0f;

  velAngHist[velAngIdx] = angleNow;

  velAngIdx++;
  if (velAngIdx > kVelWinTicks)
  {
    velAngIdx = 0;
  }

  if (velAngFill < kVelWinTicks + 1u)
  {
    velAngFill++;   /* 缓冲还没填满，这一拍给不出有效差分 */
  }
  else
  {
    v = (angleNow - velAngHist[velAngIdx]) / kVelWinSeconds;
  }

  return v;
}
}

/** @brief 【功能】上电初始化：配置 CAN 过滤器与中断，启动 1 kHz 控制定时器 */
void ControlTaskInit(void)
{
  CAN_FilterTypeDef filter;

  /* CAN 接收过滤器（掩码模式）：只比较 ID 的高 7 位，放行 0x200~0x20F，
   * 覆盖 GM6020 的反馈 ID（0x205~0x20B）；左移 5 位是因为标准帧 ID 存在寄存器高 16 位。 */
  filter.FilterIdHigh         = (0x200 << 5);
  filter.FilterIdLow          = 0x0000;
  filter.FilterMaskIdHigh     = (0x7F0 << 5);
  filter.FilterMaskIdLow      = 0x0000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation     = ENABLE;
  filter.SlaveStartFilterBank = 14;                    /* CAN2 从第 14 组开始，这里只用 CAN1 */

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* 开启 FIFO0 新消息中断，每收到一帧就调用 HAL_CAN_RxFifo0MsgPendingCallback() */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  traj.Reset();
  velPid.Reset();
  posPid.Reset();

  /* 启动 TIM6 中断，从这一刻起 MainTask() 每 1 ms 被调用一次 */
  HAL_TIM_Base_Start_IT(&htim6);
}

/** @brief 【功能】1 kHz 控制主循环，在 TIM6 周期中断回调里被调用 */
void MainTask(void)
{
  /* ① 喂狗：放在循环开头，程序卡死或跑飞则 0.8 s 后自动复位 */
  HAL_IWDG_Refresh(&hiwdg);

  /* ② 推进轨迹，取本拍的目标值（两种模式各取所需） */
  traj.Step();
  float velRef = traj.VelRef();
  float posRef = traj.PosRef();

  /* ③ 反馈超时检查：收到 CAN 反馈时清零，一直收不到说明通信断了，必须停机 */
  if (timeout < 0xFFFFFFFFu)
  {
    timeout++;   /* 加上限，防止溢出回绕 */
  }
  if (timeout > kFeedbackTimeout)
  {
    Stop();
    return;   /* 本拍不再往下算 */
  }

  /* ③ 安全检查之二：电机过温 */
  if (motor.Temp() > kTempLimit)
  {
    Stop();
    return;
  }

  float cmd;      /* 本拍控制量给定：电压模式是电压，电流模式是电流 */
  float velCmd;   /* 本拍希望电机达到的转速，摩擦前馈用它判断方向 */

  /* 【调试】反馈有间断时重置差分窗口，否则会拿断线前后的角度相减算出假速度，把控制量顶到限幅；
   * 正常通信时 timeout 只在 0/1 之间跳，>2 说明真的漏帧了。 */
  if (timeout > 2u)
  {
    velAngFill = 0;
  }

  /* 【功能】取本拍累计角度：位置环的反馈量，两种模式都要用 */
  float angNow = motor.Angle();

  /* 【调试】再用角度差分算一份更准的实际转速，此值只喂 dbgActual，不进 PID */
  float velFb = AngleVelocity(angNow);

  /* 刻意的分工：内环 PID 用驱动器自报的转速 motor.Vel()，它虽不准但即时（最多滞后一个
   * 反馈周期），高增益下闭环反而更稳；角度差分更准但有半个窗口的滞后，会吃掉相位裕度，
   * 所以只用于调试输出 dbgActual——它才反映电机真正在怎么转。 */
  if (mode == CTRL_MODE_POS)
  {
    /* 位置模式（双环串级）：外环位置误差 -> 目标转速，内环转速误差 -> 控制量 */
    velCmd = posPid.Calc(posRef, angNow);
    cmd = velPid.Calc(velCmd, motor.Vel());
  }
  else
  {
    /* 速度模式（单环）：目标转速与实际转速之差直接送进速度环 */
    velCmd = velRef;
    cmd = velPid.Calc(velRef, motor.Vel());
  }

  /* 【功能】叠加摩擦前馈，让目标速度过零时也有力 */
  cmd += FrictionFeedforward(velCmd);

  /* 最后再限一次幅：PID 内部已有输出限幅，这里是双保险，也保证 (int16_t) 转换不溢出 */
  if (cmd > kCmdMax)
  {
    cmd = kCmdMax;
  }
  else if (cmd < kCmdMin)
  {
    cmd = kCmdMin;
  }

  /* 【调试】给 Ozone 留一份目标/实际快照，位置模式单位 rad，速度模式单位 rad/s；删掉不影响控制 */
  if (mode == CTRL_MODE_POS)
  {
    dbgTarget = posRef;
    dbgActual = angNow;
  }
  else
  {
    dbgTarget = velRef;
    dbgActual = velFb;
  }

  /* ⑤ 下发控制量并发帧 */
  motor.SetVoltage((int16_t)cmd);
  SendControlFrame();
}

/** @brief 【功能】CAN 接收回调，在 HAL_CAN_RxFifo0MsgPendingCallback() 里被调用（运行在中断中）
 *  @param std_id 收到报文的标准帧 ID
 *  @param data   8 字节数据 */
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
  /* 总线上可能有多个电机，只处理 ID 匹配自己那个的帧 */
  if (std_id == motor.RxId())
  {
    motor.Decode(data);
    timeout = 0;   /* 收到反馈，超时计数清零 */
  }
}

/** @brief 【调试】切换控制模式：复位 PID 与轨迹，避免带着上个模式的积分乱跑。
 * 本工程用 kDefaultMode 固定模式，运行时切换只是留着的备用调参入口 */
void ControlTaskSetMode(uint32_t m)
{
  mode = m;
  velPid.Reset();
  posPid.Reset();
  traj.Reset();
  timeout = 0;
}

/** @brief 【调试】取当前控制模式，只给调试查看用 */
uint32_t ControlTaskGetMode(void)
{
  return mode;
}

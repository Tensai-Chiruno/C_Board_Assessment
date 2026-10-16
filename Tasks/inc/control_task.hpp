/**
 * @file  : control_task.hpp
 * @brief : 控制主任务（1 kHz 控制循环 + CAN 收发 + 看门狗）；CTRL_MODE_VEL 跟踪速度正弦，CTRL_MODE_POS 跟踪位置正弦
 */

#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CTRL_MODE_VEL   (0u)
#define CTRL_MODE_POS   (1u)

/** @brief 上电初始化：配置 CAN 过滤器与中断，启动 1 kHz 控制定时器 */
void ControlTaskInit(void);

/** @brief 1 kHz 控制入口，在 TIM6 周期中断回调中调用 */
void MainTask(void);

/** @brief CAN 反馈回调（HAL_CAN_RxFifo0MsgPendingCallback）；@param std_id 标准帧 ID，@param data 8 字节数据 */
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);

/** @brief 切换控制模式（CTRL_MODE_VEL / CTRL_MODE_POS） */
void ControlTaskSetMode(uint32_t mode);

/** @brief 取当前控制模式 */
uint32_t ControlTaskGetMode(void);

#ifdef __cplusplus
}
#endif

#endif

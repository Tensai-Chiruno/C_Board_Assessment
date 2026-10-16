/**
 * @file  : control_task_hooks.c
 * @brief : HAL 弱回调接线
 *
 * HAL 回调都是 __weak 空实现，覆写后转发给控制任务，
 * 这样不必改动 CubeMX 生成的中断文件。
 */

#include "main.h"
#include "can.h"
#include "tim.h"

#include "control_task.hpp"

/** @brief TIM6 周期中断（1 kHz）-> MainTask() */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    MainTask();
  }
}

/** @brief CAN1 接收回调（FIFO0）
 *  只处理 hcan1 的帧，其余 CAN 实例不走这里 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan != &hcan1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  if (rx_header.IDE == CAN_ID_STD)
  {
    CanFeedbackCallback(rx_header.StdId, rx_data);
  }
}


#ifndef _FDCAN_INIT_H
#define _FDCAN_INIT_H

#include "fdcan.h"

void FDCAN_Filter_Init(FDCAN_HandleTypeDef* hfdcan);
uint8_t FDCAN_Sent(FDCAN_HandleTypeDef* hfdcan,uint32_t id, uint8_t * data);
uint8_t FDCAN_EXT_Sent(FDCAN_HandleTypeDef* hfdcan,uint32_t ext_id, uint8_t * data);
uint32_t CAN_Receive_DataFrame(FDCAN_HandleTypeDef *hcan, uint8_t *buf);
#endif

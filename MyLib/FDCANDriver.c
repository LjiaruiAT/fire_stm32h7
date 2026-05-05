#include "FDCANDriver.h"

void FDCAN_Filter_Init(FDCAN_HandleTypeDef* hfdcan)
{   
    FDCAN_FilterTypeDef Sta_filter; 
Sta_filter.IdType = FDCAN_STANDARD_ID;
Sta_filter.FilterIndex = 0;
Sta_filter.FilterType = FDCAN_FILTER_RANGE;  // 使用范围过滤
Sta_filter.FilterID1 = 0x201; // 最小ID
Sta_filter.FilterID2 = 0x204; // 最大ID
Sta_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
HAL_FDCAN_ConfigFilter(hfdcan,&Sta_filter);
	
	
	FDCAN_FilterTypeDef Ext_filter;
	Ext_filter.IdType = FDCAN_EXTENDED_ID;
	Ext_filter.FilterIndex = 1;
	Ext_filter.FilterType = FDCAN_FILTER_MASK;
	Ext_filter.FilterID1 = 0x00;
  Ext_filter.FilterID2 = 0x00;
  Ext_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 
	HAL_FDCAN_ConfigFilter(hfdcan,&Ext_filter);	
    
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); 
  HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
  //HAL_FDCAN_Start(hfdcan);
}

uint8_t FDCAN_Sent(FDCAN_HandleTypeDef* hfdcan,uint32_t id, uint8_t * data)
{
     FDCAN_TxHeaderTypeDef TxHeader;
    
    TxHeader.Identifier = id;                               //识别标识符
    TxHeader.IdType = FDCAN_STANDARD_ID;                    //ID类型
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;                //帧类型
    TxHeader.DataLength = 8;                                //数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x00;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
        return 1;
    return 0;
}

uint8_t FDCAN_EXT_Sent(FDCAN_HandleTypeDef* hfdcan,uint32_t ext_id, uint8_t * data)
{
	FDCAN_TxHeaderTypeDef TxHeader;
	    TxHeader.Identifier = ext_id;                               //识别标识符
    TxHeader.IdType = FDCAN_EXTENDED_ID;                    //ID类型
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;                //帧类型
    TxHeader.DataLength = 8;                                //数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x00;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
        return 1;
    return 0;
}

uint32_t CAN_Receive_DataFrame(FDCAN_HandleTypeDef *hcan, uint8_t *buf) {
    FDCAN_RxHeaderTypeDef CAN_Rx = { 0 };
    HAL_FDCAN_GetRxMessage(hcan, (hcan->Instance == FDCAN1) ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1, &CAN_Rx, buf);

        return CAN_Rx.Identifier;

}

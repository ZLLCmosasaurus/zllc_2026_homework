#include "motor.h"
#include "can.h" 

Motor_State_TypeDef motor_state[4] = {0};
uint8_t can_rx_flag = 0;

void CAN_Motor_Init(void) {
    CAN_FilterTypeDef can_filter;
    
    can_filter.FilterActivation = ENABLE;
    can_filter.FilterBank = 0;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan, &can_filter);

    HAL_CAN_Start(&hcan);
}

void CAN_Receive_Init(void) {
    // 开启CAN接收FIFO0中断
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_Send_Motor_Cmd(uint8_t motor_id, float current) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    int16_t current_int = (int16_t)(current);

    tx_header.StdId = 0x200;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;

    if (motor_id < 4) {
        tx_data[motor_id * 2] = (current_int >> 8) & 0xFF;  // 高8位
        tx_data[motor_id * 2 + 1] = current_int & 0xFF;     // 低8位
    }

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204) {
        uint8_t motor_id = rx_header.StdId - 0x201; 
 
        motor_state[motor_id].angle_raw = (rx_data[0] << 8) | rx_data[1];
        motor_state[motor_id].speed_raw = (rx_data[2] << 8) | rx_data[3];
        motor_state[motor_id].current_raw = (rx_data[4] << 8) | rx_data[5];
        motor_state[motor_id].temp = rx_data[6];

        motor_state[motor_id].angle = (float)motor_state[motor_id].angle_raw / 8191.0f * 360.0f;
        motor_state[motor_id].speed = (float)motor_state[motor_id].speed_raw;
        motor_state[motor_id].current = (float)motor_state[motor_id].current_raw / 1000.0f;
        
        can_rx_flag = 1;
    }
}


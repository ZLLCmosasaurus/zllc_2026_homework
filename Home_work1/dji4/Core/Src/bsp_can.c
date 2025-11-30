#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;//开启过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;                        //(0x201 << 5); CAN_3508_M1_ID = 0x201
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;                                   //(0x7FF << 5);  
    can_filter_st.FilterBank = 0; //
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//
	  HAL_CAN_Start(&hcan);//启动can外设
    HAL_CAN_ConfigFilter(&hcan, &can_filter_st);//智能筛选器
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//开启接收中断

   
   

}

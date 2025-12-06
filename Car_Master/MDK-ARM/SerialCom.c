#include "SerialCom.h"

uint8_t rx_data_packet[10];
uint8_t rx_data;        // 数据字节
Serial_RxStateTypeDef SerialCom_RxState = SERIAL_RX_IDLE;

/**
 * @brief 编码器串口初始化
 */
void SerialCom_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
}

/**
 * @brief 串口接收中断
 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
    
    
//if(huart->Instance == USART2)
  //{
       
     /*  static uint8_t rx_packet_index = 0;     // 数据包的具体位置

      if(rx_data == 0xFE && SerialCom_RxState == SERIAL_RX_IDLE) // 接收包头
        {
       rx_packet_index = 0;
          memset(rx_data_packet, 0, 10);      // 清空数据防止污染
          SerialCom_RxState = SERIAL_RX_BODY;
         rx_data_packet[rx_packet_index] = rx_data;
          rx_packet_index ++;
            
     }
        
        else if(SerialCom_RxState == SERIAL_RX_BODY)   // 接收数据字节
      {
            rx_data_packet[rx_packet_index] = rx_data;
            if(rx_packet_index >= 9)
           {
                SerialCom_RxState = SERIAL_RX_IDLE;
           }
        }
		
		else if(SerialCom_RxState == SERIAL_RX_IDLE && rx_data != 0xFE)
		{
            memset(rx_data_packet, 0, 10);      // 清空数据防止污染		
            		}

      HAL_UART_Receive_IT(huart, &rx_data, 1);
    }
}*/
/**
 * @brief 解析协议
 * @param dataX 在对应位置放你的速度变量
 * @return 1=成功，0=失败
 */
uint8_t SerialCom_GetEncoder(int16_t *data1, int16_t *data2, int16_t *data3, int16_t *data4)
{
    // 非定义的数据包
    if(rx_data_packet[0] != 0xFE || rx_data_packet[9] != 0xEE)
    {
        return 0;
    }

    // 解析正常数据包
    *data1 = ((int16_t)(rx_data_packet[1]) << 8) | rx_data_packet[2];
    *data2 = ((int16_t)(rx_data_packet[3]) << 8) | rx_data_packet[4];
    *data3 = ((int16_t)(rx_data_packet[5]) << 8) | rx_data_packet[6];
    *data4 = ((int16_t)(rx_data_packet[7]) << 8) | rx_data_packet[8];

    return 1;
}

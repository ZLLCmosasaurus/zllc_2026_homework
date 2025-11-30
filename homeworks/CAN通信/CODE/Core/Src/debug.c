#include "debug.h"
#include "usart.h"

const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
uint8_t tx_step       = 0; // 0: 空闲, 1: 正在发送数据, 2: 正在发送 tail

void print_debug(float *data, size_t length)
{
    tx_step = 1;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, length * sizeof(float));
}

/**
 * @brief  Tx Transfer completed callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance) {
        if (tx_step == 1) {
            // 步骤 1 完成：浮点数据发送完毕
            tx_step = 2; // 标记为发送 tail

            // 启动第二次 DMA 传输：发送 tail
            HAL_UART_Transmit_DMA(&huart1, tail, 4);
        } else if (tx_step == 2) {
            // 步骤 2 完成：tail 数据发送完毕
            tx_step = 0; // 标记为完全空闲，可以开始下一个任务
        }
    }
}

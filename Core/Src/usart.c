/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_control.h"

// 串口接收缓冲区
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_byte;
static uint16_t uart_rx_index = 0;
static uint8_t uart_bin_mode = 0;
static uint16_t uart_bin_expected_len = 0;

// 控制模式标识：0=PS2手柄模式, 1=上位机控制模式
extern uint8_t control_mode;

/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = UART_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = UART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, UART_TX_Pin|UART_RX_Pin);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  printf重定向
  */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/**
  * @brief  启动串口中断接收
  */
void UART_IT_Enable(void)
{
	HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
}

/**
  * @brief  串口接收中断回调函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		// 先尝试存储字节
		if(uart_rx_index < UART_RX_BUFFER_SIZE)
		{
			uart_rx_buffer[uart_rx_index++] = uart_rx_byte;
		}
		else
		{
			// 缓冲区满，复位
			uart_rx_index = 0;
			uart_bin_mode = 0;
			uart_bin_expected_len = 0;
		}

		// 二进制帧模式识别：头部0xAA,0x55
		if(!uart_bin_mode && uart_rx_index >= 2 && uart_rx_buffer[0] == 0xAA && uart_rx_buffer[1] == 0x55)
		{
			uart_bin_mode = 1;
		}

		// 如果是二进制模式，长度字节到达后计算整帧长度
		if(uart_bin_mode && uart_bin_expected_len == 0 && uart_rx_index >= 3)
		{
			uint8_t N = uart_rx_buffer[2]; // N = CMD + Payload 字节数
			uart_bin_expected_len = 4 + N;  // 2头 + 1长 + (CMD+Payload) + 1校验
			if(uart_bin_expected_len > UART_RX_BUFFER_SIZE)
			{
				// 非法长度，复位
				uart_rx_index = 0;
				uart_bin_mode = 0;
				uart_bin_expected_len = 0;
			}
		}

		// 如果达到期望长度，校验并处理二进制帧
		if(uart_bin_mode && uart_bin_expected_len > 0 && uart_rx_index >= uart_bin_expected_len)
		{
			// 校验
			uint8_t N = uart_rx_buffer[2];
			uint8_t cmd = uart_rx_buffer[3];
			uint8_t checksum = uart_rx_buffer[3 + N];
			uint32_t sum = N + cmd;
			for(uint16_t i = 0; i < (uint16_t)(N - 1); i++)
			{
				sum += uart_rx_buffer[4 + i];
			}
			if(((uint8_t)sum) == checksum)
			{
				UART_Process_Command_Array(uart_rx_buffer, uart_bin_expected_len);
			}
			else
			{
				printf("ERROR: Checksum mismatch (bin)\r\n");
			}
			// 复位缓冲
			uart_rx_index = 0;
			uart_bin_mode = 0;
			uart_bin_expected_len = 0;
		}
		else if(!uart_bin_mode)
		{
			// 文本命令：以 \n 或 \r 为行尾
			if(uart_rx_byte == '\n' || uart_rx_byte == '\r')
			{
				if(uart_rx_index > 0)
				{
					uart_rx_buffer[uart_rx_index - 1] = '\0';  // 将行尾替换为字符串结束符
					UART_Process_Command();  // 处理文本命令
					uart_rx_index = 0;  // 重置索引
				}
			}
		}

		// 继续接收
		HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
	}
}

/**
  * @brief  处理上位机命令
  * @命令格式：
  *   手动控制：  CMD:MOVE,<方向>,<速度>   例如: CMD:MOVE,F,500
  *   自动寻路：  CMD:AUTO,<路径>         例如: CMD:AUTO,F500,R90,F300
  *   停止：      CMD:STOP
  *   查询状态：  CMD:STATUS
  *   复位编码器：CMD:RESET_ENC
  *   切换模式：  CMD:MODE,<0|1>  (0=PS2, 1=UART)
  */
void UART_Process_Command(void)
{
	char *token;
	char buffer[UART_RX_BUFFER_SIZE];
	strcpy(buffer, (char*)uart_rx_buffer);
	
	// 解析命令头
	token = strtok(buffer, ":");
	if(token == NULL || strcmp(token, "CMD") != 0)
	{
		printf("ERROR: Invalid command format\r\n");
		return;
	}
	
	// 解析命令类型
	token = strtok(NULL, ",");
	if(token == NULL)
	{
		printf("ERROR: No command type\r\n");
		return;
	}
	
	// 处理移动命令
	if(strcmp(token, "MOVE") == 0)
	{
		char *direction = strtok(NULL, ",");
		char *speed_str = strtok(NULL, ",");
		
		if(direction == NULL || speed_str == NULL)
		{
			printf("ERROR: Invalid MOVE command\r\n");
			return;
		}
		
		int16_t speed = atoi(speed_str);
		
		// 限制速度范围
		if(speed < MIN_PWM) speed = MIN_PWM;
		if(speed > MAX_PWM) speed = MAX_PWM;
		
		if(strcmp(direction, "F") == 0)  // Forward
		{
			Car_Forward(speed);
			printf("OK: Moving forward at %d\r\n", speed);
		}
		else if(strcmp(direction, "B") == 0)  // Backward
		{
			Car_Backward(speed);
			printf("OK: Moving backward at %d\r\n", speed);
		}
		else if(strcmp(direction, "L") == 0)  // Turn Left
		{
			Car_TurnLeft(speed);
			printf("OK: Turning left at %d\r\n", speed);
		}
		else if(strcmp(direction, "R") == 0)  // Turn Right
		{
			Car_TurnRight(speed);
			printf("OK: Turning right at %d\r\n", speed);
		}
		else
		{
			printf("ERROR: Invalid direction\r\n");
		}
	}
	// 处理停止命令
	else if(strcmp(token, "STOP") == 0)
	{
		Car_Stop();
		printf("OK: Car stopped\r\n");
	}
	// 处理状态查询命令
	else if(strcmp(token, "STATUS") == 0)
	{
		UART_SendCarStatus();
	}
	// 复位编码器
	else if(strcmp(token, "RESET_ENC") == 0)
	{
		Encoder_ResetCounts();
		printf("OK: Encoder reset\r\n");
	}
	// 切换控制模式
	else if(strcmp(token, "MODE") == 0)
	{
		char *mode_str = strtok(NULL, ",");
		if(mode_str == NULL)
		{
			printf("ERROR: No mode specified\r\n");
			return;
		}
		
		uint8_t mode = atoi(mode_str);
		if(mode == 0 || mode == 1)
		{
			control_mode = mode;
			printf("OK: Mode set to %d (%s)\r\n", mode, mode == 0 ? "PS2" : "UART");
		}
		else
		{
			printf("ERROR: Invalid mode (0=PS2, 1=UART)\r\n");
		}
	}
	else
	{
		printf("ERROR: Unknown command\r\n");
	}
}

/**
  * @brief  发送小车状态（数组帧）
  * @frame 格式: AA 55 LEN CMD PAYLOAD CHECK
  *         CMD=0x84, PAYLOAD: LeftEnc(int32 LE), RightEnc(int32 LE), Mode(uint8)
  */
void UART_SendCarStatus_Array(void)
{
	int32_t left_enc = Encoder_GetLeftCount();
	int32_t right_enc = Encoder_GetRightCount();
	uint8_t mode = control_mode;

	uint8_t frame[16];
	uint8_t idx = 0;
	frame[idx++] = 0xAA;
	frame[idx++] = 0x55;
	uint8_t payload_len = 4 + 4 + 1; // 两个int32 + 1字节模式
	uint8_t N = 1 + payload_len;     // CMD + payload
	frame[idx++] = N;
	frame[idx++] = 0x84;             // 响应命令码
	// LeftEnc (LE)
	frame[idx++] = (uint8_t)(left_enc & 0xFF);
	frame[idx++] = (uint8_t)((left_enc >> 8) & 0xFF);
	frame[idx++] = (uint8_t)((left_enc >> 16) & 0xFF);
	frame[idx++] = (uint8_t)((left_enc >> 24) & 0xFF);
	// RightEnc (LE)
	frame[idx++] = (uint8_t)(right_enc & 0xFF);
	frame[idx++] = (uint8_t)((right_enc >> 8) & 0xFF);
	frame[idx++] = (uint8_t)((right_enc >> 16) & 0xFF);
	frame[idx++] = (uint8_t)((right_enc >> 24) & 0xFF);
	// Mode
	frame[idx++] = mode;
	// checksum
	uint32_t sum = N + 0x84;
	for(uint8_t i = 0; i < payload_len; i++) sum += frame[4 + i];
	frame[idx++] = (uint8_t)sum;

	HAL_UART_Transmit(&huart3, frame, idx, 0xFFFF);
}

/**
  * @brief  处理上位机数组指令
  * @frame 格式: AA 55 LEN CMD PAYLOAD CHECK
  * @note   LEN = CMD+PAYLOAD 字节数, CHECK = (LEN + CMD + PAYLOAD字节求和) & 0xFF
  * @commands
  *   CMD=0x01 MOVE: [dir:uint8][speed:uint16 LE]
  *   CMD=0x02 STOP: 无负载
  *   CMD=0x03 MODE: [mode:uint8]  (0=PS2,1=UART)
  *   CMD=0x04 STATUS: 请求状态（数组响应）
  *   CMD=0x05 RESET_ENC: 无负载
  */
void UART_Process_Command_Array(const uint8_t* frame, uint16_t len)
{
	if(len < 5) { printf("ERROR: Frame too short\r\n"); return; }
	if(frame[0] != 0xAA || frame[1] != 0x55) { printf("ERROR: Bad header\r\n"); return; }
	uint8_t N = frame[2];
	if(len != (uint16_t)(4 + N)) { printf("ERROR: Length mismatch\r\n"); return; }
	uint8_t cmd = frame[3];
	const uint8_t* payload = &frame[4];
	uint8_t checksum = frame[3 + N];
	uint32_t sum = N + cmd;
	for(uint16_t i = 0; i < (uint16_t)(N - 1); i++) sum += payload[i];
	if(((uint8_t)sum) != checksum) { printf("ERROR: Checksum\r\n"); return; }

	switch(cmd)
	{
		case 0x01: // MOVE
		{
			if(N != 3) { printf("ERROR: MOVE payload\r\n"); break; }
			uint8_t dir = payload[0];
			int16_t speed = (int16_t)(payload[1] | (payload[2] << 8));
			if(speed < MIN_PWM) speed = MIN_PWM;
			if(speed > MAX_PWM) speed = MAX_PWM;
			switch(dir)
			{
				case 0x00: Car_Forward(speed); printf("OK: F %d\r\n", speed); break;
				case 0x01: Car_Backward(speed); printf("OK: B %d\r\n", speed); break;
				case 0x02: Car_TurnLeft(speed); printf("OK: TL %d\r\n", speed); break;
				case 0x03: Car_TurnRight(speed); printf("OK: TR %d\r\n", speed); break;
				case 0x04: Car_Move_Left(speed); printf("OK: ML %d\r\n", speed); break;
				case 0x05: Car_Move_Right(speed); printf("OK: MR %d\r\n", speed); break;
				case 0x06: Car_Move_FL(speed); printf("OK: FL %d\r\n", speed); break;
				case 0x07: Car_Move_FR(speed); printf("OK: FR %d\r\n", speed); break;
				case 0x08: Car_Move_BL(speed); printf("OK: BL %d\r\n", speed); break;
				case 0x09: Car_Move_BR(speed); printf("OK: BR %d\r\n", speed); break;
				default: printf("ERROR: DIR\r\n"); break;
			}
			break;
		}
		case 0x02: // STOP
		{
			if(N != 1) { printf("ERROR: STOP payload\r\n"); break; }
			Car_Stop();
			printf("OK: STOP\r\n");
			break;
		}
		case 0x03: // MODE
		{
			if(N != 2) { printf("ERROR: MODE payload\r\n"); break; }
			uint8_t mode = payload[0];
			if(mode == 0 || mode == 1)
			{
				control_mode = mode;
				printf("OK: MODE %d\r\n", mode);
			}
			else
			{
				printf("ERROR: MODE val\r\n");
			}
			break;
		}
		case 0x04: // STATUS request
		{
			if(N != 1) { printf("ERROR: STATUS payload\r\n"); break; }
			UART_SendCarStatus_Array();
			break;
		}
		case 0x05: // RESET_ENC
		{
			if(N != 1) { printf("ERROR: RESET payload\r\n"); break; }
			Encoder_ResetCounts();
			printf("OK: RESET_ENC\r\n");
			break;
		}
		default:
			printf("ERROR: Unknown CMD 0x%02X\r\n", cmd);
			break;
	}
}

/**
  * @brief  发送小车状态
  */
void UART_SendCarStatus(void)
{
	int32_t left_enc = Encoder_GetLeftCount();
	int32_t right_enc = Encoder_GetRightCount();
	
	printf("STATUS: LeftEnc=%ld, RightEnc=%ld, Mode=%d\r\n", 
	       left_enc, right_enc, control_mode);
}
/* USER CODE END 1 */

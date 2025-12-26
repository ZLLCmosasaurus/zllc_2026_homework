#include "stm32f10x.h"

void LED_init()
{
GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO ,ENABLE );  //开启AFIO的时钟，准备给重映射时钟供电
	GPIO_PinRemapConfig (GPIO_Remap_SWJ_JTAGDisable ,ENABLE );		//开启SWJ的重映射
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET);
	GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_SET);
	GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_SET);
  GPIO_WriteBit(GPIOA,GPIO_Pin_15,Bit_SET);
}


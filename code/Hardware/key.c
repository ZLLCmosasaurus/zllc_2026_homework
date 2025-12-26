#include "stm32f10x.h"
#include "Delay.h"

void Key_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStrucure; 
	GPIO_InitStrucure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStrucure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStrucure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStrucure);
}

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 0)
	{	
		Delay_ms(20);//key1按键按下
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 0);
		Delay_ms(20);
		KeyNum = 1;
	}
		if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 0)
	{	
		Delay_ms(20);//key2按键按下
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 0);
		Delay_ms(20);
		KeyNum = 2;
	}
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0)
	{	
		Delay_ms(20);//key3按键按下
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0);
		Delay_ms(20);
		KeyNum = 3;
	}
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0)
	{	
		Delay_ms(20);//key4按键按下
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0);
		Delay_ms(20);
		KeyNum = 4;
	}
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) == 0)
	{	
		Delay_ms(20);//触碰按键按下
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) == 0);
		Delay_ms(20);
		KeyNum = 5;
	}
	return KeyNum;
}

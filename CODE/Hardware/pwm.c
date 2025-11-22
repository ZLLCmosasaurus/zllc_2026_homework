#include "pwm.h"

void pwm_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE );//为时钟二赋能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP  ;  //为什么要复用？  因为PA2,PA3是由时钟2控制高低电平的
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);  //PA2 PA3初始化
	
	
	TIM_TimeBaseInitTypeDef timbasestruct;
	timbasestruct .TIM_CounterMode =TIM_CounterMode_Up ;
	timbasestruct .TIM_Period =1000-1;
	timbasestruct .TIM_Prescaler =720-1;
	timbasestruct .TIM_RepetitionCounter =0;
	TIM_TimeBaseInit (TIM2 ,&timbasestruct );
	
	TIM_OCInitTypeDef pwmint;
	pwmint.TIM_Pulse =0;
	pwmint .TIM_OCMode =TIM_OCMode_PWM1 ;
	pwmint .TIM_OCPolarity =TIM_OCPolarity_High ;
	pwmint .TIM_OutputState =TIM_OutputState_Enable ;
	TIM_OC3Init (TIM2 ,&pwmint );   //oc3是什么意思  output channel3，即通道3
	TIM_OC4Init (TIM2 ,&pwmint );
	
	TIM_Cmd(TIM2,ENABLE );

}

void pwm_setfor(int ccr)
{
	TIM_SetCompare3(TIM2 ,ccr);  //设置定时器2上的通道3

}

void pwm_setback(int ccr)
{
	TIM_SetCompare4(TIM2 ,ccr);

}


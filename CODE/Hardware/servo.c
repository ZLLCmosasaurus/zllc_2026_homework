#include "stm32f10x.h"
#include "servo.h"

servo_para servo1_para={150,100,150,240};  //前后    pwm   leftlimit   middle   rightlimit 
servo_para servo2_para={150,100,150,250};		//上下	
servo_para servo3_para={150,50,150,250};  //爪子

void servo_init()
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12;           //舵机1
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP   ;  //为什么不是复用？  因为PB12是CPU控制高低电平的
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);  

	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;						//舵机2
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP   ;  //为什么不是复用？  因为PB13是CPU控制高低电平的
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);  

	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP   ;  //为什么不是复用？  因为PB14是CPU控制高低电平的
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE );//为时钟1赋能(我一开始没看到前面是APB1导致调了好久的代码)
	TIM_TimeBaseInitTypeDef timbasestruct;
	timbasestruct.TIM_CounterMode =TIM_CounterMode_Up ;
	timbasestruct.TIM_Period =10-1;   //10us一次，记录2000次，充当20ms周期
	timbasestruct.TIM_Prescaler =72-1;
	timbasestruct.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit (TIM1 ,&timbasestruct );
	TIM_ClearFlag (TIM1 ,TIM_FLAG_Update ); //把标志位清零
	TIM_ITConfig (TIM1 ,TIM_IT_Update ,ENABLE );//开启定时器1中断
	

	NVIC_InitTypeDef nvic_initstruct;
	nvic_initstruct.NVIC_IRQChannel=TIM1_UP_IRQn;
	nvic_initstruct.NVIC_IRQChannelPreemptionPriority=0;
	nvic_initstruct.NVIC_IRQChannelSubPriority=0;
	nvic_initstruct.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_Init(&nvic_initstruct);
	
	
	TIM_Cmd(TIM1,ENABLE );  //开启时钟1
	servo_set1(servo1_para.middle);//  归位中值
	servo_set2(servo2_para.middle);//  归位中值
	servo_set3(servo3_para.middle);//  归位中值
}

uint16_t count,servo_ccr1,servo_ccr2,servo_ccr3;

void servo_set1(int set_ccr)  //前后  
{		
		if(set_ccr<servo1_para.leftlimit)   //不小于左极限(看机械结构定)
			set_ccr=servo1_para.leftlimit;
		if(set_ccr>servo1_para.rightlimit)  //不超过右极限
			set_ccr=servo1_para.rightlimit;
		servo_ccr1=set_ccr;
}

void servo_set2(int set_ccr)  //上下
{		
		if(set_ccr<servo2_para.leftlimit)
			set_ccr=servo2_para.leftlimit;
		if(set_ccr>servo2_para.rightlimit)
			set_ccr=servo2_para.rightlimit;
		servo_ccr2=set_ccr;
}

void servo_set3(int set_ccr)  //夹子
{		
		if(set_ccr<servo3_para.leftlimit)  
			set_ccr=servo3_para.leftlimit;
		if(set_ccr>servo3_para.rightlimit)
			set_ccr=servo3_para.rightlimit;
		servo_ccr3=set_ccr;
}





void TIM1_UP_IRQHandler()
{
	if(TIM_GetFlagStatus (TIM1 ,TIM_FLAG_Update )==SET )
	{
		TIM_ClearFlag (TIM1 ,TIM_FLAG_Update );  //清除中断标志
		count++;
		if(count>2000)  //模拟20ms的pwm输出
		{
			count=0;
		}
		if(count<servo_ccr1 )  //舵机1模拟ccr，即输出对应高低电压
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET); //高电压
		}
		else   
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET);//低电压
		}
		if(count<servo_ccr2 )  //舵机2模拟ccr，即输出对应高低电压
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET); //高电压
		}
		else   
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_RESET);//低电压
		}
		if(count<servo_ccr3 )  //舵机3模拟ccr，即输出对应高低电压
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_SET); //高电压
		}
		else   
		{
			GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_RESET);//低电压
		}
		
	}

}


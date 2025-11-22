#include "stm32f10x.h"
#include "motor.h"
void motor_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4,ENABLE );//打开时钟二和四
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP  ;  //为什么要复用？  因为PA2,PA3是由时钟2控制高低电平的
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);  
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP  ;  //为什么要复用？  因为PA2,PA3是由时钟2控制高低电平的
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);  
	
	
	
	TIM_TimeBaseInitTypeDef timbasestruct;
	timbasestruct .TIM_CounterMode =TIM_CounterMode_Up ;
	timbasestruct .TIM_Period =1000-1;
	timbasestruct .TIM_Prescaler =720-1;  //100hz
	timbasestruct .TIM_RepetitionCounter =0;
	TIM_TimeBaseInit (TIM2 ,&timbasestruct );
	TIM_TimeBaseInit (TIM4 ,&timbasestruct );
	
	
	TIM_OCInitTypeDef pwmint;
	pwmint.TIM_Pulse =0;
	pwmint .TIM_OCMode =TIM_OCMode_PWM1 ;
	pwmint .TIM_OCPolarity =TIM_OCPolarity_High ;
	pwmint .TIM_OutputState =TIM_OutputState_Enable ;
	TIM_OC1Init (TIM2 ,&pwmint );   
	TIM_OC2Init (TIM2 ,&pwmint );
	TIM_OC3Init (TIM2 ,&pwmint );  
	TIM_OC4Init (TIM2 ,&pwmint );
	TIM_OC1Init (TIM4 ,&pwmint );   
	TIM_OC2Init (TIM4 ,&pwmint );
	TIM_OC3Init (TIM4 ,&pwmint );  
	TIM_OC4Init (TIM4 ,&pwmint );
	
	TIM_Cmd(TIM2,ENABLE );
	TIM_Cmd(TIM4,ENABLE );
	

}

void carforward(int speed)
{
	motor1_F(1,speed);
	motor2_F(0,speed);
	motor3_F(0,speed);
	motor4_F(1,speed);
}

void carbackward(int speed)
{
	motor1_F(0,speed);
	motor2_F(1,speed);
	motor3_F(1,speed);
	motor4_F(0,speed);
}

void car_transleft(int speed)
{	
	motor1_F(1,speed);
	motor2_F(1,speed);
	motor3_F(0,speed);
	motor4_F(0,speed);

}


void car_transright(int speed)
{	
	motor1_F(0,speed);   //反转
	motor2_F(0,speed);	 //正转
	motor3_F(1,speed);	//反转
	motor4_F(1,speed);   //正转

}

void car_turnleft(int speed)
{
	motor1_F(0,speed);   //反转
	motor2_F(0,speed);		//正转
	motor3_F(0,speed);		//正转
	motor4_F(0,speed);   //反转


}


void car_turnright(int speed)
{
	motor1_F(1,speed);   //正转
	motor2_F(1,speed);		//反转
	motor3_F(1,speed);		//反转
	motor4_F(1,speed);   //正转


}

/****************************************************************************/
/****************************************************************************/
/*电机15
PA0-TIM2CHANNEL1-B
PA1-TIM2CHANNEL2-F
*/
void motor1_F(int dir,int speed)
{
	if(dir==1)  //反转
	{
		TIM_SetCompare1(TIM2, speed);
		TIM_SetCompare2(TIM2, 0);
	
	}
	else
	{
		TIM_SetCompare1(TIM2,0 );
		TIM_SetCompare2(TIM2, speed);
	}
	

}
/*电机16
PA2-TIM2CHANNEL3-B
PA3-TIM2CHANNEL4-F
*/
void motor2_F(int dir,int speed)
{
	if(dir==1)
	{
		TIM_SetCompare3(TIM2, speed);
		TIM_SetCompare4(TIM2, 0);
	}
	else
	{
		TIM_SetCompare3(TIM2, 0);
		TIM_SetCompare4(TIM2, speed);
	
	}

}

/*电机18
PB6-TIM4CHANNEL1-B
PB7-TIM4CHANNEL2-F
*/
void motor3_F(int dir,int speed)
{
	if(dir==1)
	{
		TIM_SetCompare1(TIM4, speed);
		TIM_SetCompare2(TIM4, 0);
	}
	else
	{
		TIM_SetCompare1(TIM4, 0);
		TIM_SetCompare2(TIM4, speed);
	
	}

}




/*电机17
PB8-TIM4CHANNEL3-B
PB9-TIM4CHANNEL4-F
*/
void motor4_F(int dir,int speed)
{
	if(dir==1)
	{
		TIM_SetCompare3(TIM4, speed);
		TIM_SetCompare4(TIM4, 0);
	}
	else
	{
		TIM_SetCompare3(TIM4, 0);
		TIM_SetCompare4(TIM4, speed);
	
	}

}


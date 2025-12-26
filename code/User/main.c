
#include "stm32f10x.h"
#include "key.h"
#include "pwm.h"
#include "ps2.h"
#include "led.h"
#include "motor.h"
#include "servo.h"
#include "Delay.h"
uint8_t ps2_key,ps2mode,ps2lastmode;

#define unit_pwm 2;   //PWM的改变单元

extern servo_para servo1_para;
extern servo_para servo2_para;
extern servo_para servo3_para;


int main()
{
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2 );
	PS2_Init();
	LED_init();  //为什么？不把PA15改成gpio输出，按键就无效
	motor_init();
	servo_init(); 
	while(1)
	{
		
		ps2_key=ps2_key_serch ();
		
		ps2mode=ps2_mode_get ();
		
		if(ps2mode !=ps2lastmode)//如果模式切换，让小车停下
		{
			carforward (0);  //停车
			
		}
		
		if(ps2mode ==PSB_GREENLIGHT_MODE ) //绿灯模式，按键有效，高速
		{
				if(ps2_key)  //有值，说明有按键按下
				{
					
					//左侧按键--控制车轮
						if(ps2_get_key_state(PSB_PAD_UP))
						{
							carforward(1000);
							
						}
						if(ps2_get_key_state(PSB_PAD_DOWN ))
						{
							carbackward (1000);
						}
						if(ps2_get_key_state(PSB_PAD_LEFT ))
						{
							car_transleft(1000);
						}
						if(ps2_get_key_state(PSB_PAD_RIGHT ))
						{
							car_transright (1000);																																																																																																																																																																																																																																																																								
						}
						
					//右侧按键--控制舵机
						if(ps2_get_key_state(PSB_GREEN ))   //上--前
						{
							servo1_para.pwm+=unit_pwm;
							servo_set1(servo1_para.pwm);																																																																																																																																																																																																																																																																								
						}
						if(ps2_get_key_state(PSB_BLUE ))   //下--后
						{
							servo1_para.pwm-=unit_pwm;
							servo_set1(servo1_para.pwm);																																																																																																																																																																																																																																																																								
						}
						if(ps2_get_key_state(PSB_PINK )) //左--上
						{
							servo2_para.pwm+=unit_pwm;
							servo_set2(servo2_para.pwm);																																																																																																																																																																																																																																																																							
						}
						if(ps2_get_key_state(PSB_RED ))  //右--下
						{
							servo2_para.pwm-=unit_pwm;
							servo_set2(servo2_para.pwm);																																																																																																																																																																																																																																																																								
						}
						if(ps2_get_key_state(PSB_R1)) //R1--夹
						{
							servo3_para.pwm-=unit_pwm;
							servo_set3(servo3_para.pwm);																																																																																																																																																																																																																																																																								
						}
						if(ps2_get_key_state(PSB_R2 ))  //R2--松
						{
							servo3_para.pwm+=unit_pwm;
							servo_set3(servo3_para.pwm);																																																																																																																																																																																																																																																																								
						}
						if(ps2_get_key_state(PSB_L1)) //L1--左转弯
						{
							car_turnleft(1000);	
						}																																																																																																																																																																																																																																																																							
						if(ps2_get_key_state(PSB_L2 ))  //L2--右转弯
						{
							car_turnright(1000);																																																																																																																																																																																																																																																																								
						}
						
						
				}
				else   
				{
					carforward (0);   
				}
				
				
				
		}
		else if(ps2mode ==PSB_REDLIGHT_MODE) //红灯模式，摇杆有效，低速
		{
				if(ps2_get_anolog_data(PSS_LY)<0x6f)  //左杆向上
				{
	
					carforward (500);
				}
				else if(ps2_get_anolog_data(PSS_LY)>0x8f)		//左杆向下
				{
		
					carbackward (500);
				}
				else if(ps2_get_anolog_data(PSS_LX)<0x6f)		//左杆向左
				{
			
					car_turnleft(500);
				}
				else if(ps2_get_anolog_data(PSS_LX)>0x8f)		//左杆向右
				{
					
					car_turnright(500);
				}
				else if(ps2_get_anolog_data(PSS_RY)<0x6f)  //右杆向上
				{
					servo1_para.pwm+=unit_pwm;
					servo_set1(servo1_para.pwm);
				}
				else if(ps2_get_anolog_data(PSS_RY)>0x8f)		//右杆向下
				{
					servo1_para.pwm-=unit_pwm;
					servo_set1(servo1_para.pwm);
				}
				else if(ps2_get_anolog_data(PSS_RX)<0x6f)		//右杆向左
				{
					servo2_para.pwm+=unit_pwm;
					servo_set2(servo2_para.pwm);
				}
				else if(ps2_get_anolog_data(PSS_RX)>0x8f)		//右杆向右
				{
					servo2_para.pwm-=unit_pwm;
					servo_set2(servo2_para.pwm);
				}
				else
				{
					carforward (0);
				}
		}
		ps2lastmode=ps2mode;
		Delay_ms (20);//非常非常重要，不然手柄和中转器传输有问题，不能无间隔查询
	
	
	}

}


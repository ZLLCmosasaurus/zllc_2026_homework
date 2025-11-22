#include "servo_control.h"
#include <stdio.h>

// 舵机当前角度 (私有变量)
static uint8_t servo_angles[4] = {180, 180, 180, 180};

// 上次更新时间 (用于控制更新频率)
static uint32_t last_update_time = 0;

/**
  * @brief  初始化舵机控制 (TIM1)
  * @param  无
  * @retval 无
 * @note   需要先在CubeMX中配置TIM1为PWM模式（舵机用）
 *         Prescaler=71, Period=19999 -> 50Hz
  */
void Servo_Init(void)
{
	// 启动TIM1的4个PWM通道（4舵机方案）
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PA8 - 舵机1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // PA9 - 舵机2
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // PA10 - 舵机3
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // PA11 - 舵机4
	
	// 初始化所有舵机(180度)
	Servo_ResetAll();
}

/**
  * @brief  设置舵机角度
  * @param  servo_id: 舵机编号 (SERVO_1~SERVO_4)
  * @param  angle: 角度 (0~180度)
  * @retval 无
  */
void Servo_SetAngle(Servo_ID_t servo_id, uint8_t angle)
{
	// 角度限幅
	if(angle > SERVO_ANGLE_MAX) 
		angle = SERVO_ANGLE_MAX;
	
	// 角度转换为脉宽 (0度=500us, 180度=2500us)
	// 公式: pulse = 500 + (angle / 180) * 2000
	uint16_t pulse_us = SERVO_PULSE_MIN + 
	                    ((uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / SERVO_ANGLE_MAX;
	
	Servo_SetPulse(servo_id, pulse_us);
	servo_angles[servo_id] = angle;
}

/**
  * @brief  设置舵机脉宽 (微秒)
  * @param  servo_id: 舵机编号
  * @param  pulse_us: 脉宽 (500~2500微秒)
  * @retval 无
  */
void Servo_SetPulse(Servo_ID_t servo_id, uint16_t pulse_us)
{
	uint32_t channel;
	
	// 选择对应的定时器通道
	switch(servo_id)
	{
		case SERVO_1: channel = TIM_CHANNEL_1; break;  // PA8
		case SERVO_2: channel = TIM_CHANNEL_2; break;  // PA9
		case SERVO_3: channel = TIM_CHANNEL_3; break;  // PA10
		case SERVO_4: channel = TIM_CHANNEL_4; break;  // PA11
		default: return;
	}
	
	// 设置PWM比较值 (脉宽直接对应计数值,因为定时器频率是1MHz)
	__HAL_TIM_SET_COMPARE(&htim1, channel, pulse_us);
}

/**
  * @brief  获取舵机当前角度
  * @param  servo_id: 舵机编号
  * @retval 当前角度 (0~180)
  */
uint8_t Servo_GetAngle(Servo_ID_t servo_id)
{
	if(servo_id >= 4) return 0;
	return servo_angles[servo_id];
}

/**
  * @brief  复位所有舵机到中位
  * @param  无
  * @retval 无
  */
void Servo_ResetAll(void)
{
	// 4舵机方案，复位所有4个舵机
	for(uint8_t i = 0; i < 4; i++)
	{
		Servo_SetAngle((Servo_ID_t)i, SERVO_ANGLE_MID);
	}
}

/**
  * @brief  PS2手柄控制舵机
  * @param  btn1: 按键组1 (Bit4=UP, Bit5=RIGHT, Bit6=DOWN, Bit7=LEFT)
  * @param  btn2: 按键组2 (Bit4=△, Bit5=○, Bit6=×, Bit7=□)
  * @retval 无
 * @note   PS2手柄按键按下时对应位为0 (低电平有效)
 *         每组2个按键控制1个舵机的正反转 (4舵机方案):
 *         - 舵机1: UP(正转) + DOWN(反转)
 *         - 舵机2: LEFT(正转) + RIGHT(反转)
 *         - 舵机3: △(正转) + ×(反转)
 *         - 舵机4: □(正转) + ○(反转)
  */
void PS2_Control_Servo(uint8_t btn1, uint8_t btn2)
{
	// PS2按键位定义 (根据ax_ps2.h中的数据格式)
	#define BTN_UP     (1<<4)  // Bit4
	#define BTN_RIGHT  (1<<5)  // Bit5
	#define BTN_DOWN   (1<<6)  // Bit6
	#define BTN_LEFT   (1<<7)  // Bit7
	
	#define BTN_TRI    (1<<4)  // △ (Triangle)
	#define BTN_CIR    (1<<5)  // ○ (Circle)
	#define BTN_CRO    (1<<6)  // × (Cross)
	#define BTN_SQU    (1<<7)  // □ (Square)
	
	uint32_t current_time = HAL_GetTick();
	
	// 限制更新频率,避免舵机抖动
	if(current_time - last_update_time < SERVO_UPDATE_MS) 
		return;
	
	last_update_time = current_time;
	
	// ====== 舵机1: UP(正转) + DOWN(反转) ======
	if(!(btn1 & BTN_UP))  // UP键按下,舵机1正转(角度增加)
	{
		uint8_t current_angle = servo_angles[SERVO_1];
		if(current_angle + SERVO_STEP_ANGLE <= SERVO_ANGLE_MAX)
		{
			Servo_SetAngle(SERVO_1, current_angle + SERVO_STEP_ANGLE);
		}
	}
	if(!(btn1 & BTN_DOWN))  // DOWN键按下,舵机1反转(角度减少)
	{
		uint8_t current_angle = servo_angles[SERVO_1];
		if(current_angle >= SERVO_STEP_ANGLE)
		{
			Servo_SetAngle(SERVO_1, current_angle - SERVO_STEP_ANGLE);
		}
	}
	
	// ====== 舵机2: LEFT(正转) + RIGHT(反转) ======
	if(!(btn1 & BTN_LEFT))  // LEFT键按下,舵机2正转(角度增加)
	{
		uint8_t current_angle = servo_angles[SERVO_2];
		if(current_angle + SERVO_STEP_ANGLE <= SERVO_ANGLE_MAX)
		{
			Servo_SetAngle(SERVO_2, current_angle + SERVO_STEP_ANGLE);
		}
	}
	
	if(!(btn1 & BTN_RIGHT))  // RIGHT键按下,舵机2反转(角度减少)
	{
		uint8_t current_angle = servo_angles[SERVO_2];
		if(current_angle >= SERVO_STEP_ANGLE)
		{
			Servo_SetAngle(SERVO_2, current_angle - SERVO_STEP_ANGLE);
		}
	}
	
	// ====== 舵机3: △(正转) + ×(反转) ======
	if(!(btn2 & BTN_TRI))  // △键按下,舵机3正转(角度增加)
	{
		uint8_t current_angle = servo_angles[SERVO_3];
		if(current_angle + SERVO_STEP_ANGLE <= SERVO_ANGLE_MAX)
		{
			Servo_SetAngle(SERVO_3, current_angle + SERVO_STEP_ANGLE);
		}
	}
	if(!(btn2 & BTN_CRO))  // ×键按下,舵机3反转(角度减少)
	{
		uint8_t current_angle = servo_angles[SERVO_3];
		if(current_angle >= SERVO_STEP_ANGLE)
		{
			Servo_SetAngle(SERVO_3, current_angle - SERVO_STEP_ANGLE);
		}
	}
	
	// ====== 舵机4: □(正转) + ○(反转) ======
	if(!(btn2 & BTN_SQU))  // □键按下,舵机4正转(角度增加)
	{
		uint8_t current_angle = servo_angles[SERVO_4];
		if(current_angle + SERVO_STEP_ANGLE <= SERVO_ANGLE_MAX)
		{
			Servo_SetAngle(SERVO_4, current_angle + SERVO_STEP_ANGLE);
		}
	}
	if(!(btn2 & BTN_CIR))  // ○键按下,舵机4反转(角度减少)
	{
		uint8_t current_angle = servo_angles[SERVO_4];
		if(current_angle >= SERVO_STEP_ANGLE)
		{
			Servo_SetAngle(SERVO_4, current_angle - SERVO_STEP_ANGLE);
		}
	}
}

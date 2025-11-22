#include "motor_control.h"
#include <stdlib.h>  // for abs() function

// 电机速度和编码器变量
static int32_t left_encoder_total = 0;
static int32_t right_encoder_total = 0;
static int32_t left_encoder_last = 0;
static int32_t right_encoder_last = 0;

// 里程计变量
static float left_distance_mm = 0.0f;   // 左侧行驶距离(mm)
static float right_distance_mm = 0.0f;  // 右侧行驶距离(mm)

// 速度测量变量
static float left_speed_mms = 0.0f;     // 左侧速度(mm/s)
static float right_speed_mms = 0.0f;    // 右侧速度(mm/s)
static int32_t left_encoder_prev = 0;   // 上次测量的左侧编码器值
static int32_t right_encoder_prev = 0;  // 上次测量的右侧编码器值
static uint32_t speed_last_update_time = 0;  // 上次速度更新时间

/**
  * @brief  初始化电机和编码器
  * @param  无
  * @retval 无
  */
void Motor_Init(void)
{
	// 确保所有PWM通道都设置为0
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);  // PB6 - 左前轮
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);  // PB7 - 左后轮
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);  // PB8 - 右前轮
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);  // PB9 - 右后轮
	
	// 启动PWM TIM4的4个通道，分别控制4个车轮
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // PB6 - 左前轮
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // PB7 - 左后轮
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  // PB8 - 右前轮
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  // PB9 - 右后轮
	
	// 启动编码器TIM2和TIM3
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	// 确保所有电机停止
	Car_Stop();
	
	// 再次确保所有PWM通道都设置为0
	//问题，轮子自转
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);  // PB6 - 左前轮
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);  // PB7 - 左后轮
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);  // PB8 - 右前轮
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);  // PB9 - 右后轮
	
	Encoder_ResetCounts();
}

/**
  * @brief  麦克纳姆轮四轮独立控制
  * @param  lf_pwm: 左前轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @param  rf_pwm: 右前轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @param  lb_pwm: 左后轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @param  rb_pwm: 右后轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @retval 无
  * @note   四轮独立PWM控制，支持麦克纳姆轮全向移动
  */
void Mecanum_Control(int16_t lf_pwm, int16_t rf_pwm, int16_t lb_pwm, int16_t rb_pwm)
{
	// 左前轮控制 (TIM4_CH1 = PB6)
	if(lf_pwm > MIN_PWM)
	{
		LF_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, lf_pwm);
	}
	else if(lf_pwm < -MIN_PWM)
	{
		LF_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, -lf_pwm);
	}
	else 
	{
		LF_STOP();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	}
	
	// 左后轮控制 (TIM4_CH2 = PB7)
	if(lb_pwm > MIN_PWM)
	{
		LB_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, lb_pwm);
	}
	else if(lb_pwm < -MIN_PWM)
	{
		LB_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -lb_pwm);
	}
	else 
	{
		LB_STOP();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	}
	
	// 右前轮控制 (TIM4_CH3 = PB8)
	if(rf_pwm > MIN_PWM)
	{
		RF_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, rf_pwm);
	}
	else if(rf_pwm < -MIN_PWM)
	{
		RF_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -rf_pwm);
	}
	else 
	{
		RF_STOP();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	}
	
	// 右后轮控制 (TIM4_CH4 = PB9)
	if(rb_pwm > MIN_PWM)
	{
		RB_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, rb_pwm);
	}
	else if(rb_pwm < -MIN_PWM)
	{
		RB_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -rb_pwm);
	}
	else 
	{
		RB_STOP();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	}
}

/**
  * @brief  停止小车
  * @param  无
  * @retval 无
  */
void Car_Stop(void)
{
	LF_STOP();
	RF_STOP();
	LB_STOP();
	RB_STOP();
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

/**
  * @brief  小车前进（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Forward(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 前进: LF+ RF+ LB+ RB+
	Mecanum_Control(speed, speed, speed, speed);
}

/**
  * @brief  小车后退（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Backward(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 后退: LF- RF- LB- RB-
	Mecanum_Control(-speed, -speed, -speed, -speed);
}

/**
  * @brief  小车原地左转（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_TurnLeft(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 原地左转: LF- RF+ LB- RB+
	Mecanum_Control(-speed, speed, -speed, speed);
}

/**
  * @brief  小车原地右转（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_TurnRight(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 原地右转: LF+ RF- LB+ RB-
	Mecanum_Control(speed, -speed, speed, -speed);
}

/**
  * @brief  小车左平移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_Left(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 左平移: LF- RF+ LB+ RB-
	Mecanum_Control(-speed, speed, speed, -speed);
}

/**
  * @brief  小车右平移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_Right(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 右平移: LF+ RF- LB- RB+
	Mecanum_Control(speed, -speed, -speed, speed);
}

/**
  * @brief  小车左前斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_FL(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 左前斜移: LF0 RF+ LB+ RB0
	Mecanum_Control(0, speed, speed, 0);
}

/**
  * @brief  小车右前斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_FR(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 右前斜移: LF+ RF0 LB0 RB+
	Mecanum_Control(speed, 0, 0, speed);
}

/**
  * @brief  小车左后斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_BL(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 左后斜移: LF- RF0 LB0 RB-
	Mecanum_Control(-speed, 0, 0, -speed);
}

/**
  * @brief  小车右后斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_BR(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 右后斜移: LF0 RF- LB- RB0
	Mecanum_Control(0, -speed, -speed, 0);
}

/**
  * @brief  将摇杆值映射为PWM值
  * @param  joystick_val: 摇杆值 (0~255, 中点127)
  * @retval PWM值 (-MAX_PWM ~ MAX_PWM)
  */
int16_t Map_Joystick_To_PWM(uint8_t joystick_val)
{
	int16_t pwm_val;
	
	// 摇杆值 > 127 + 死区
	if(joystick_val > (127 + DEAD_ZONE))
	{
		pwm_val = (joystick_val - 127) * MAX_PWM / 128;
		return (pwm_val > MAX_PWM) ? MAX_PWM : pwm_val;
	}
	// 摇杆值 < 127 - 死区
	else if(joystick_val < (127 - DEAD_ZONE))
	{
		pwm_val = (127 - joystick_val) * MAX_PWM / 128;
		return (pwm_val > MAX_PWM) ? -MAX_PWM : -pwm_val;
	}
	// 死区
	else 
	{
		return 0;
	}
}

/**
  * @brief  速度滤波器更新
  * @param  left_filtered: 左侧滤波后速度
  * @param  right_filtered: 右侧滤波后速度
  * @param  left_target: 左侧目标速度
  * @param  right_target: 右侧目标速度
  * @retval 无
  */
void Speed_Filter_Update(int16_t* left_filtered, int16_t* right_filtered, int16_t left_target, int16_t right_target)
{
	// 一阶滤波
	*left_filtered = (int16_t)(0.3 * left_target + 0.7 * (*left_filtered));
	*right_filtered = (int16_t)(0.3 * right_target + 0.7 * (*right_filtered));
}

/**
  * @brief  获取左侧编码器计数值
  * @param  无
  * @retval 编码器计数值
  */
int32_t Encoder_GetLeftCount(void)
{
	int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	int16_t delta = current_count - left_encoder_last;
	
	if(delta > 32767) delta -= 65536;
	else if(delta < -32767) delta += 65536;		//2^16
	
	left_encoder_total += delta;
	left_encoder_last = current_count;
	
	return left_encoder_total;
}

/**
  * @brief  获取右侧编码器计数值
  * @param  无
  * @retval 编码器计数值
  */
int32_t Encoder_GetRightCount(void)
{
	int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	int16_t delta = current_count - right_encoder_last;
	
	if(delta > 32767) delta -= 65536;
	else if(delta < -32767) delta += 65536;
	
	right_encoder_total += delta;
	right_encoder_last = current_count;
	
	return right_encoder_total;
}

/**
  * @brief  重置编码器计数值
  * @param  无
  * @retval 无
  */
void Encoder_ResetCounts(void)
{
	left_encoder_total = 0;
	right_encoder_total = 0;
	left_encoder_last = __HAL_TIM_GET_COUNTER(&htim2);
	right_encoder_last = __HAL_TIM_GET_COUNTER(&htim3);
}

/**
  * @brief  获取左侧行驶距离
  * @param  无
  * @retval 距离(mm)
  */
float Odometer_GetLeftDistance(void)
{
	// 更新左侧编码器计数
	Encoder_GetLeftCount();
	
	// 将脉冲数转换为距离(mm)
	left_distance_mm = (float)left_encoder_total / PULSES_PER_MM;
	
	return left_distance_mm;
}

/**
  * @brief  获取右侧行驶距离
  * @param  无
  * @retval 距离(mm)
  */
float Odometer_GetRightDistance(void)
{
	// 更新右侧编码器计数
	Encoder_GetRightCount();
	
	// 将脉冲数转换为距离(mm)
	right_distance_mm = (float)right_encoder_total / PULSES_PER_MM;
	
	return right_distance_mm;
}

/**
  * @brief  获取总行驶距离(左右轮平均)
  * @param  无
  * @retval 距离(mm)
  */
float Odometer_GetTotalDistance(void)
{
	float left_dist = Odometer_GetLeftDistance();
	float right_dist = Odometer_GetRightDistance();
	
	// 返回左右轮的平均距离
	return (left_dist + right_dist) / 2.0f;
}

/**
  * @brief  重置里程计
  * @param  无
  * @retval 无
  */
void Odometer_Reset(void)
{
	Encoder_ResetCounts();
	left_distance_mm = 0.0f;
	right_distance_mm = 0.0f;
	left_encoder_prev = 0;
	right_encoder_prev = 0;
	speed_last_update_time = HAL_GetTick();
}

/**
  * @brief  更新速度测量(需要定时调用，建议100ms)
  * @param  无
  * @retval 无
  */
void Speed_Update(void)
{
	uint32_t current_time = HAL_GetTick();
	uint32_t delta_time = current_time - speed_last_update_time;
	
	// 避免除零
	if(delta_time == 0) return;
	
	// 获取当前编码器总计数
	int32_t left_count = Encoder_GetLeftCount();
	int32_t right_count = Encoder_GetRightCount();
	
	// 计算脉冲增量
	int32_t left_delta_pulses = left_count - left_encoder_prev;
	int32_t right_delta_pulses = right_count - right_encoder_prev;
	
	// 转换为距离增量(mm)
	float left_delta_distance = (float)left_delta_pulses / PULSES_PER_MM;
	float right_delta_distance = (float)right_delta_pulses / PULSES_PER_MM;
	
	// 计算速度(mm/s)
	left_speed_mms = (left_delta_distance * 1000.0f) / (float)delta_time;
	right_speed_mms = (right_delta_distance * 1000.0f) / (float)delta_time;
	
	// 保存当前值供下次计算
	left_encoder_prev = left_count;
	right_encoder_prev = right_count;
	speed_last_update_time = current_time;
}

/**
  * @brief  获取左侧速度
  * @param  无
  * @retval 速度(mm/s)
  */
float Speed_GetLeft(void)
{
	return left_speed_mms;
}

/**
  * @brief  获取右侧速度
  * @param  无
  * @retval 速度(mm/s)
  */
float Speed_GetRight(void)
{
	return right_speed_mms;
}

/**
  * @brief  PS2手柄控制麦克纳姆轮小车（支持全向移动和按键旋转）
  * @param  ljoy_lr: 左摇杆左右值 (0~255, 0=左, 127=中, 255=右)
  * @param  ljoy_ud: 左摇杆上下值 (0~255, 0=上, 127=中, 255=下)
  * @param  btn1: 按键1状态（Bit1=JOYR, Bit2=JOYL，按下时对应位为1）
  * @retval 无
  * @说明  控制逻辑：
  *        1. 优先级最高：按键原地旋转
  *           - 按住JOYL（Bit2=1）：原地左转
  *           - 按住JOYR（Bit1=1）：原地右转
  *        2. 其次：左摇杆控制全向移动
  *           - 上推(0)：前进
  *           - 下拉(255)：后退
  *           - 左推(0)：左平移
  *           - 右推(255)：右平移
  *           - 斜向推：斜向移动（麦克纳姆轮特性）
  *        3. 麦克纳姆轮运动学公式：
  *           LF = forward + strafe
  *           RF = forward - strafe
  *           LB = forward - strafe
  *           RB = forward + strafe
  */
void PS2_Control_Car(uint8_t ljoy_lr, uint8_t ljoy_ud, uint8_t btn1)
{
	// 定义按键位：Bit1=JOYR, Bit2=JOYL
	#define BTN_JOYR  (1<<1)  // Bit1
	#define BTN_JOYL  (1<<2)  // Bit2
	
	int16_t forward_speed = 0;  // 前后速度
	int16_t strafe_speed = 0;   // 平移速度
	int16_t lf_pwm = 0, rf_pwm = 0, lb_pwm = 0, rb_pwm = 0;
	
	// 只有在有效的手柄模式下才处理控制指令
	// 0x73 = 红灯模式(数字模式), 0x41 = 绿灯模式(模拟模式)
	static uint8_t last_valid_mode = 0;
	
	// 检查是否为有效的手柄模式
	if(last_valid_mode == 0 && (ljoy_lr != 127 || ljoy_ud != 127))
	{
		// 如果是第一次接收到有效数据，记录有效模式
		last_valid_mode = 1;
	}
	
	// 如果还没有接收到有效的手柄数据，不执行任何动作
	if(last_valid_mode == 0)
	{
		Car_Stop();
		return;
	}
	
	// 优先级最高：检查按键原地旋转
	// 注意：PS2按键按下时对应位为1（因为btn1已经取反）
	if(btn1 & BTN_JOYL)  // 按下JOYL：原地左转
	{
		Car_TurnLeft(550);  // 使用固定速度旋转
		return;
	}
	else if(btn1 & BTN_JOYR)  // 按下JOYR：原地右转
	{
		Car_TurnRight(550);
		return;
	}
	
	// 将摇杆值转换为PWM值
	// ljoy_ud: 0=上(前进), 127=中点, 255=下(后退)
	// 需要反转：上推(0)应该是正速度(前进)，下拉(255)应该是负速度(后退)
	forward_speed = -Map_Joystick_To_PWM(ljoy_ud);
	
	// ljoy_lr: 0=左(左平移), 127=中点, 255=右(右平移)
	// 左推(0)应该是负速度(左平移)，右推(255)应该是正速度(右平移)
	strafe_speed = Map_Joystick_To_PWM(ljoy_lr);
	
	// 麦克纳姆轮运动学公式（修正版）：
	// 前进：LF+ RF+ LB+ RB+  -> forward>0, strafe=0
	// 后退：LF- RF- LB- RB-  -> forward<0, strafe=0
	// 左平移：LF- RF+ LB+ RB-  -> forward=0, strafe<0
	// 右平移：LF+ RF- LB- RB+  -> forward=0, strafe>0
	// 原地左转：LF- RF+ LB- RB+  -> forward=0, 需要特殊处理
	// 原地右转：LF+ RF- LB+ RB-  -> forward=0, 需要特殊处理
	//
	// 标准麦克纳姆轮公式：
	// LF = forward + strafe
	// RF = forward - strafe
	// LB = forward - strafe
	// RB = forward + strafe
	
	lf_pwm = forward_speed + strafe_speed;
	rf_pwm = forward_speed - strafe_speed;
	lb_pwm = forward_speed - strafe_speed;
	rb_pwm = forward_speed + strafe_speed;
	
	// PWM限幅
	if(lf_pwm > MAX_PWM) lf_pwm = MAX_PWM;
	if(lf_pwm < -MAX_PWM) lf_pwm = -MAX_PWM;
	if(rf_pwm > MAX_PWM) rf_pwm = MAX_PWM;
	if(rf_pwm < -MAX_PWM) rf_pwm = -MAX_PWM;
	if(lb_pwm > MAX_PWM) lb_pwm = MAX_PWM;
	if(lb_pwm < -MAX_PWM) lb_pwm = -MAX_PWM;
	if(rb_pwm > MAX_PWM) rb_pwm = MAX_PWM;
	if(rb_pwm < -MAX_PWM) rb_pwm = -MAX_PWM;
	
	// 控制四个轮子
	Mecanum_Control(lf_pwm, rf_pwm, lb_pwm, rb_pwm);
}

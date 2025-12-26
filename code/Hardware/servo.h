#ifndef __SERVO_H_
#define __SERVO_H_


#include "stm32f10x.h"
void servo_init(void);
void servo_set1(int set_ccr);
void servo_set2(int set_ccr);
void servo_set3(int set_ccr);


typedef struct servo
{
	uint16_t pwm;
	uint16_t leftlimit;
	uint16_t middle;
	uint16_t rightlimit;
}servo_para;
#endif


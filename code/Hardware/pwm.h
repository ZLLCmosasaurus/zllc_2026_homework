#ifndef  __pwm_H_
#define  __pwm_H_

#include "stm32f10x.h"

void pwm_init(void);
void pwm_setfor(int ccr);
void pwm_setback(int ccr);

#endif


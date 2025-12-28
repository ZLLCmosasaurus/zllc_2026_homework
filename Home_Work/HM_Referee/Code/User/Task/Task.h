#ifndef _TASK_H_
#define _TASK_H_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif
void Task_Init();
void Task_Loop();

#ifdef __cplusplus
}
#endif

#endif // _TASK_H_

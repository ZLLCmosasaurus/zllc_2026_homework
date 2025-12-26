#ifndef TASK__H_
#define TASK__H_


#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

//extern uint16_t pwmVal;
/* Exported function declarations --------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif
 
  void Task_Init();
  void Task_Loop(); 
 
#ifdef __cplusplus
}
#endif


#endif    
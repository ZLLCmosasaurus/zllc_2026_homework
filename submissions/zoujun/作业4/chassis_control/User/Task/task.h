#ifndef __TASK_H
#define __TASK_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif
 
  void Task_Init();
  void Task_Loop(); 
 
#ifdef __cplusplus
}
#endif

#endif // !__TASK_H
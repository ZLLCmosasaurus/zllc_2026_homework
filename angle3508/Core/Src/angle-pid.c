#include "main.h"  
#include "CAN_receive.h"
#include "angle-pid.h"
#include <stdlib.h>
extern fp32 yaw_angle;
extern int ecd_sum;
extern motor_measure_t *motor1_data;
fp32 angle_calc(void)
{
	if (abs(motor1_data->ecd - motor1_data->last_ecd )> 4096) //(ecd last_ecd) distance larger than half cir
	{
		if(motor1_data->ecd < motor1_data->last_ecd) //ecd have gone zore 
		{
			ecd_sum+=8192+motor1_data->ecd - motor1_data->last_ecd;//correct direction
		
		}
		else//ecd have not gone zore
		{
			ecd_sum-=8192-motor1_data->ecd + motor1_data->last_ecd;//reverse direction
		
		}
	}
	else
	{
		ecd_sum+=motor1_data->ecd - motor1_data->last_ecd; //correct direction no mater + or -
	}
	
	motor1_data->last_ecd=motor1_data->ecd ;//get_motor_measure()have done this process
	
	/*if(abs(ecd_sum)>= 8192*19)//because rate=19:1,limit one cir
	{
		ecd_sum=0; //return to zero
	}*/
		yaw_angle=ecd_sum *0.000040367915470674769203107568102385f;//ecd divert to yaw_angle(pi)
		//yaw_angle=ecd_sum*2*pi/8192.0/19.0;//ecd divert to yaw_angle(pi)
		return yaw_angle;
}
	
	
	
	
	


		



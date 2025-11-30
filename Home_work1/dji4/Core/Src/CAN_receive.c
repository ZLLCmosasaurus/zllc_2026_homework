#include "CAN_receive.h"
#include "main.h"


extern CAN_HandleTypeDef hcan;

pid_struct_t pid_motor1;

#define get_motor_measure(ptr, data)                                    \
{                                                                       \
    (ptr)->last_ecd = (ptr)->ecd;                                       \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                \
    (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);           \
    (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);       \
    (ptr)->temperate = (data)[6];                                       \
}
//measure反馈部分
static motor_measure_t motor_chassis[4];

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

// PID初始化函数
void PID_Motor1_Init(void) {
    pid_motor1.kp = 7.8f;    // 比例系数
    pid_motor1.ki = 0.0f;    // 积分系数
    pid_motor1.kd = 0.8f;    // 微分系数
    pid_motor1.i_max = 500.0f;     // 积分限幅
    pid_motor1.out_max = 16000.0f; // 输出限幅（3508最大电流估计值）
    pid_motor1.ref = 0.0f;
    pid_motor1.fdb = 0.0f;
    pid_motor1.err = 0.0f;
    pid_motor1.err_last = 0.0f;
    pid_motor1.p_out = 0.0f;
    pid_motor1.i_out = 0.0f;
    pid_motor1.d_out = 0.0f;
    pid_motor1.output = 0.0f;
}

// PID计算函数
float pid_calc(pid_struct_t *pid, float ref, float fdb) {
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err_last = pid->err;
    pid->err = pid->ref - pid->fdb;
    pid->p_out = pid->kp * pid->err;
    pid->i_out += pid->ki * pid->err;
    pid->d_out = pid->kd * (pid->err - pid->err_last);
    LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
    return pid->output;
}

// 目标转速设置函数
void Set_Motor1_TargetSpeed(float rpm) {
    pid_motor1.ref = rpm;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
     cnn+=1;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];


    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);


    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            
            uint8_t i = rx_header.StdId - CAN_3508_M1_ID;
            
            get_motor_measure(&motor_chassis[i], rx_data);
            pid_motor1.fdb = (float)motor_chassis[i].speed_rpm; 
            break;
        }
        default:
            break;
    }
}


/*void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;

    // ??CAN???(DJI 3508??????)
    chassis_tx_message.StdId = 0x700;  // ??????ID
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;     // ????8??

    // ?????0(????)
    memset(chassis_can_send_data, 0, 8);

    // ??CAN1????
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}*/

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;

    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;  
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;                  

    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = 0 >> 8;//motor2，3，4暂时改为0
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0 >> 8;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0 >> 8;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}



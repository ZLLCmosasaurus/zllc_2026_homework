# CAN电机控制项目代码详解

## 一、主函数代码详解

### 1.1 主函数初始化部分

```c
int main(void)
{
    /* USER CODE BEGIN 1 */     
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();                     

    /* USER CODE BEGIN Init */       
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();            

    /* USER CODE BEGIN SysInit */    
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();                  // 第3步：初始化GPIO，配置引脚模式
    MX_CAN_Init();                   // 第4步：初始化CAN控制器，设置波特率等参数
    MX_USART1_UART_Init();           // 第5步：初始化串口1，备用调试接口
    MX_TIM2_Init();                  // 第6步：初始化定时器2，用于1kHz中断

    /* USER CODE BEGIN 2 */        
    //CAN_Filter_Config();           // 注释掉的原始过滤器配置函数
    can_filter_mask_config(&hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0 ,0);
    // 第7步：配置CAN过滤器0
    // 参数分解：
    // CAN_FILTER(0) -> 使用第0个过滤器（共28个）
    // CAN_FIFO_0    -> 匹配的消息放入FIFO0队列
    // CAN_STDID     -> 过滤标准帧ID（11位）
    // CAN_DATA_TYPE -> 过滤数据帧（不是远程请求帧）
    // 0, 0          -> ID=0, 掩码=0，表示接收所有消息
    
    can_filter_mask_config(&hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0 ,0);
    // 第8步：配置CAN过滤器1，放入FIFO1队列
    // 两个过滤器可以接收所有消息，分别放入不同队列
    
    /*离开初始模式*/
    HAL_CAN_Start(&hcan);            // 第9步：启动CAN总线，开始正常工作
    
    /*开中断*/
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    // 第10步：使能CAN接收中断
    // CAN_IT_RX_FIFO0_MSG_PENDING -> FIFO0中有消息挂起时触发中断
    
    HAL_TIM_Base_Start_IT(&htim2);   // 第11步：启动定时器2并开启中断
    // TIM2配置为1kHz，每1ms产生一次中断
    
    // 第12步：通过RTT输出调试信息
    SEGGER_RTT_WriteString(0, "=========================================\r\n");
    SEGGER_RTT_WriteString(0, "  C620 + M3508 Motor Control Started!\r\n");
    SEGGER_RTT_WriteString(0, "  CAN: 1Mbps, ID: 0x200/0x201\r\n");
    SEGGER_RTT_WriteString(0, "  PID Control: 1kHz (TIM2 Interrupt)\r\n");
    SEGGER_RTT_WriteString(0, "=========================================\r\n");
    SEGGER_RTT_WriteString(0, "Control: Change 'target_rpm' in Ozone Watch\r\n");
    SEGGER_RTT_WriteString(0, "Default: target_rpm = 500\r\n");
    SEGGER_RTT_WriteString(0, "=========================================\r\n\r\n");

    // 第13步：发送零电流，确保电机安全启动
    send_current_to_motor(0);
    // 必须初始发送0电流，防止电机意外启动

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // 主循环中暂时没有功能，所有工作都在中断中完成
        // 第14步：可以在这里添加非实时任务
        // 例如：LED闪烁、按键检测等
        
        // uint8_t data[8] = {0,1,2,0};  // 测试数据
        // CAN_Send_Data(&hcan,0x201,data,8);  // 测试发送
    }
    /* USER CODE END 3 */
}
```

### 1.2 CAN过滤器配置函数详解

```c
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    // 断言检查，确保传入的hcan指针不为空
    assert_param(hcan != NULL);

    // 定义CAN过滤器结构体
    CAN_FilterTypeDef can_filter_init_structure;
    
    // 第1行：设置过滤器编号
    // Object_Para >> 3：右移3位，因为Object_Para的高5位表示过滤器编号
    // CAN_FILTER(0)宏定义为(0<<3)，所以右移3位得到0
    can_filter_init_structure.FilterBank = Object_Para >> 3;

    // 第2行：设置过滤模式为掩码模式
    // CAN_FILTERMODE_IDMASK：ID掩码模式
    // 另一种模式是CAN_FILTERMODE_IDLIST（列表模式）
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;

    // 第3行：判断是否为扩展帧
    // Object_Para & 0x02：检查第1位（从0开始）
    // 0x02 = 0000 0010，判断是否是扩展帧
    if ((Object_Para & 0x02))
    {   
        // 扩展帧（29位ID）处理
        // 第4行：设置为32位过滤器
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
        
        // 第5行：设置ID高16位
        // ID << 3：ID左移3位，因为扩展帧ID在寄存器中从第3位开始
        // >> 16：取高16位
        can_filter_init_structure.FilterIdHigh = (ID << 3) >> 16;
        
        // 第6行：设置ID低16位
        // ID << 3：ID左移3位
        // | (Object_Para & 0x03) << 1：添加IDE位和RTR位
        // Object_Para & 0x03取低2位：位0=RTR，位1=IDE
        // 左移1位后，这些位被放到寄存器的正确位置
        can_filter_init_structure.FilterIdLow = ID << 3 | (Object_Para & 0x03) << 1;
        
        // 第7行：设置掩码高16位
        can_filter_init_structure.FilterMaskIdHigh = (Mask_ID << 3) >> 16;
        
        // 第8行：设置掩码低16位
        // Mask_ID << 3：掩码左移3位
        // | (0x03) << 1：固定设置IDE和RTR位都需要匹配
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | (0x03) << 1;
    }
    else
    {
        // 标准帧（11位ID）处理
        // 第9行：设置为16位过滤器（标准帧用16位就够）
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_16BIT;
        
        // 第10行：设置ID高16位（标准帧不使用高16位）
        can_filter_init_structure.FilterIdHigh = 0x0000;
        
        // 第11行：设置ID低16位
        // ID << 5：标准帧ID左移5位，因为标准帧ID在寄存器中从第5位开始
        // | (Object_Para & 0x02) << 4：添加IDE位
        can_filter_init_structure.FilterIdLow = ID << 5 | (Object_Para & 0x02) << 4;
        
        // 第12行：设置掩码高16位（标准帧不使用）
        can_filter_init_structure.FilterMaskIdHigh = 0x0000;
        
        // 第13行：设置掩码低16位
        // Mask_ID << 5：掩码左移5位
        // | 0x01 << 4：设置IDE位必须匹配为0（标准帧）
        can_filter_init_structure.FilterMaskIdLow = (Mask_ID << 5) | 0x01 << 4;
    }

    // 第14行：设置使用哪个FIFO
    // (Object_Para >> 2) & 0x01：右移2位取第2位
    // 0=使用FIFO0，1=使用FIFO1
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;

    // 第15行：设置从过滤器起始编号（双CAN模式时使用）
    // STM32F103有28个过滤器，CAN1使用0-13，CAN2使用14-27
    can_filter_init_structure.SlaveStartFilterBank = 14;

    // 第16行：使能过滤器
    can_filter_init_structure.FilterActivation = ENABLE;

    // 第17行：应用过滤器配置
    if(HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure)!=HAL_OK)
    {
        Error_Handler();  // 配置失败进入错误处理
    }
}
```

## 二、电机控制模块代码详解

### 2.1 多圈角度计算函数

```c
void motor_process_data(uint16_t can_id, uint8_t* data)
{
    // 第1行：判断是否为电机反馈数据（ID 0x201-0x208）
    if (can_id >= 0x201 && can_id <= 0x208) {
        // 第2行：解析编码器值（2字节）
        // data[0] << 8：高字节左移8位
        // | data[1]：与低字节合并
        int16_t encoder = (data[0] << 8) | data[1];
        
        // 第3行：解析转速值（2字节，有符号）
        int16_t speed = (data[2] << 8) | data[3];
        
        // 第4行：解析电流值（2字节，有符号）
        int16_t current = (data[4] << 8) | data[5];
        
        // 第5行：定义编码器变化量
        int16_t delta;
        
        // 第6-12行：处理编码器溢出
        // 情况1：正转溢出（从接近最大值跳转到接近0）
        if (last_encoder > 6000 && encoder < 2000) {
            // 第7行：计算正确的delta值
            // (8192 - last_encoder)：从last_encoder到8191的距离
            // + encoder：加上从0到encoder的距离
            delta = (8192 - last_encoder) + encoder;
            // 第8行：圈数加1
            encoder_turns++;
        } 
        // 第9-12行：情况2：反转溢出（从接近0跳转到接近最大值）
        else if (last_encoder < 2000 && encoder > 6000) {
            // 第10行：计算正确的delta值（负值）
            delta = -((8192 - encoder) + last_encoder);
            // 第11行：圈数减1
            encoder_turns--;
        } 
        // 第13-14行：情况3：正常情况，没有溢出
        else {
            // 第14行：直接计算差值
            delta = encoder - last_encoder;
        }
        
        // 第15行：更新总编码器计数
        // total_encoder是32位的，可以记录非常多的圈数
        motor.total_encoder += delta;
        
        // 第16行：计算实际角度
        // encoder_turns * 360.0f：整圈的角度
        // encoder * DEGREE_PER_TICK：当前圈内的角度
        // DEGREE_PER_TICK = 360.0f / 8191 ≈ 0.04395°
        actual_degree = encoder_turns * 360.0f + encoder * DEGREE_PER_TICK;
        
        // 第17-22行：保存数据到结构体
        motor.angle = encoder;              // 原始编码器值
        motor.speed_rpm = speed;            // 转速
        motor.actual_current = current;     // 电流
        motor.angle_degree = actual_degree; // 角度值
        motor.online = 1;                   // 设置在线标志
        
        // 第23-26行：更新全局变量
        actual_rpm = speed;                     // 全局实际转速
        actual_degree = motor.angle_degree;     // 全局实际角度
        motor_online = 1;                       // 全局在线标志
        last_motor_time = HAL_GetTick();        // 记录最后通信时间
        last_encoder = encoder;                 // 保存本次编码器值
        
        // 第27-34行：调试输出（每300ms输出一次）
        static uint32_t last_debug = 0;
        if (HAL_GetTick() - last_debug > 300) {
            // 第30行：格式化输出电机状态
            SEGGER_RTT_printf(0, "MOTOR: %.1f°, %dRPM, %dA\n", 
                             actual_degree, speed, current);
            // 第31行：更新最后调试时间
            last_debug = HAL_GetTick();
        }
    }
}
```

### 2.2 位置环PID计算函数

```c
static int16_t calculate_position_pid(void)
{
    // 第1行：计算位置误差
    // target_degree：目标角度（全局变量，可以通过调试器修改）
    // actual_degree：实际角度（从电机反馈计算得到）
    float error = target_degree - actual_degree;
    
    // 第2-3行：比例项计算
    // pos_pid.kp：比例系数（当前为6.0）
    // P项作用：误差越大，输出越大，使系统快速响应
    float p_term = pos_pid.kp * error;
    
    // 第4-9行：积分项计算
    // 第5行：累加误差
    pos_pid.integral += error;
    
    // 第6-7行：积分限幅（防积分饱和）
    // 积分饱和：当误差持续存在时，积分项会一直增大
    // 导致系统响应过慢，甚至失控
    if (pos_pid.integral > 1000) pos_pid.integral = 1000;
    if (pos_pid.integral < -1000) pos_pid.integral = -1000;
    
    // 第8行：计算积分项
    // pos_pid.ki：积分系数（当前为0.122）
    // I项作用：消除稳态误差
    float i_term = pos_pid.ki * pos_pid.integral;
    
    // 第10-12行：微分项计算
    // 第10行：计算误差变化率
    // error - pos_pid.last_error：本次误差与上次误差的差值
    // pos_pid.kd：微分系数（当前为0.9）
    // D项作用：预测变化趋势，抑制超调
    float d_term = pos_pid.kd * (error - pos_pid.last_error);
    
    // 第11行：保存本次误差，用于下次计算
    pos_pid.last_error = error;
    
    // 第13行：合成PID输出（目标转速）
    // 将三个项相加得到最终输出
    float target_speed = p_term + i_term + d_term;
    
    // 第14-17行：输出限幅
    // 目标转速不能太大，否则速度环无法跟踪
    // 3000RPM是M3508电机的合理转速上限
    if (target_speed > 3000.0f) target_speed = 3000.0f;
    if (target_speed < -3000.0f) target_speed = -3000.0f;
    
    // 第18行：返回结果（强制转换为int16_t）
    return (int16_t)target_speed;
}
```

### 2.3 速度环PID计算函数

```c
static int16_t calculate_velocity_pid(int16_t target_speed)
{
    // 第1行：计算速度误差
    // target_speed：目标转速（位置环的输出或直接设置）
    // actual_rpm：实际转速（从电机反馈得到）
    int16_t error = target_speed - actual_rpm;
    
    // 第2行：比例项计算
    // vel_pid.kp：速度环比例系数（当前为3.0）
    float p_term = vel_pid.kp * error;
    
    // 第3-7行：积分项计算
    // 第4行：累加速度误差
    vel_pid.integral += error;
    
    // 第5-6行：积分限幅
    if (vel_pid.integral > 1000) vel_pid.integral = 1000;
    if (vel_pid.integral < -1000) vel_pid.integral = -1000;
    
    // 第7行：计算积分项
    // vel_pid.ki：速度环积分系数（当前为0.1）
    float i_term = vel_pid.ki * vel_pid.integral;
    
    // 第8-9行：微分项计算
    // 速度环的微分项通常较小，主要抑制转速波动
    float d_term = vel_pid.kd * (error - vel_pid.last_error);
    
    // 第10行：保存误差
    vel_pid.last_error = error;
    
    // 第11行：合成输出（电流值）
    float output = p_term + i_term + d_term;
    
    // 第12-15行：电流限幅
    // C620电调支持的电流范围：-10000 ~ +10000
    // 对应实际电流：-20A ~ +20A
    if (output > 10000) output = 10000;
    if (output < -10000) output = -10000;
    
    // 第16行：返回电流值
    return (int16_t)output;
}
```

### 2.4 PID控制更新函数

```c
void update_pid_control(void)
{
    // 第1-7行：检查电机是否在线
    if (!motor_online) {
        // 第2行：定义静态变量，记录最后发送零电流的时间
        static uint32_t last_zero_time = 0;
        
        // 第3-6行：每100ms发送一次零电流
        if (HAL_GetTick() - last_zero_time > 100) {
            send_current_to_motor(0);      // 发送零电流
            last_zero_time = HAL_GetTick(); // 更新时间
        }
        return;  // 电机不在线，直接返回
    }
    
    // 第8-9行：定义局部变量
    int16_t target_speed = 0;   // 目标转速
    int16_t current_output = 0; // 输出电流
    
    // 第10行：判断控制模式
    if (control_mode == MODE_POSITION) {
        // 位置模式：双环控制
        
        // 第12行：位置环计算目标转速
        target_speed = calculate_position_pid();
        
        // 第13行：速度环计算输出电流
        current_output = calculate_velocity_pid(target_speed);
        
        // 第15-21行：位置模式调试输出（每400ms一次）
        static uint32_t last_pos_debug = 0;
        if (HAL_GetTick() - last_pos_debug > 400) {
            // 第18行：输出位置控制相关信息
            SEGGER_RTT_printf(0, "POS: T=%.1f°, A=%.1f°, TS=%d, Out=%d\n",
                             target_degree, actual_degree, target_speed, current_output);
            last_pos_debug = HAL_GetTick();
        }
    } else {
        // 速度模式：单环控制
        
        // 第24行：直接计算速度环输出
        current_output = calculate_velocity_pid(target_rpm);
        
        // 第26-31行：速度模式调试输出
        static uint32_t last_vel_debug = 0;
        if (HAL_GetTick() - last_vel_debug > 400) {
            SEGGER_RTT_printf(0, "VEL: T=%d, A=%d, Out=%d\n",
                             target_rpm, actual_rpm, current_output);
            last_vel_debug = HAL_GetTick();
        }
    }
    
    // 第33行：发送电流到电机
    send_current_to_motor(current_output);
}
```

### 2.5 发送电流函数

```c
void send_current_to_motor(int16_t current)
{
    // 第1行：定义数据缓冲区（8字节，CAN标准帧最大数据量）
    uint8_t data[8] = {0};
    
    // 第2-3行：定义CAN发送结构体
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;  // 邮箱号，用于标识发送消息
    
    // 第4-7行：电流限幅
    // C620电调支持范围：-10000 ~ +10000
    if (current > 10000) current = 10000;
    if (current < -10000) current = -10000;
    
    // 第8行：设置CAN ID
    // 0x200：控制电机的标准ID
    tx_header.StdId = 0x200;
    
    // 第9行：扩展ID设为0（使用标准帧）
    tx_header.ExtId = 0;
    
    // 第10行：设置帧类型为标准帧
    tx_header.IDE = CAN_ID_STD;
    
    // 第11行：设置数据帧（不是远程请求帧）
    tx_header.RTR = CAN_RTR_DATA;
    
    // 第12行：设置数据长度（8字节）
    tx_header.DLC = 8;
    
    // 第13行：禁用全局时间戳
    tx_header.TransmitGlobalTime = DISABLE;
    
    // 第14-15行：打包电流数据到CAN帧
    // 电流值是有符号16位整数，需要拆分成两个字节
    data[0] = (current >> 8) & 0xFF;  // 高字节
    data[1] = current & 0xFF;         // 低字节
    // 注意：data[2]-data[7]保持为0，对应其他电机的电流控制
    
    // 第16-19行：发送CAN消息
    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &mailbox) == HAL_OK) {
        can_tx_count++;       // 发送计数器加1
        output_current = current;  // 保存当前输出电流
    }
}
```

## 三、定时器中断处理

### 3.1 定时器中断回调函数

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 第1行：判断是哪个定时器产生的中断
    if (htim->Instance == TIM2) {
        // 第2行：执行PID控制更新
        // TIM2配置为1kHz，所以每1ms执行一次
        update_pid_control();
    }
}
```

**为什么使用1kHz的频率？**
1. **电机响应速度**：M3508电机的时间常数较小，需要较高的控制频率
2. **PID计算需求**：位置环和速度环都需要及时更新
3. **系统实时性**：1ms的控制周期可以保证良好的实时性
4. **STM32能力**：STM32F103可以轻松处理1kHz的中断

### 3.2 CAN接收中断回调函数

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 第1-2行：定义接收变量
    CAN_RxHeaderTypeDef rx_header;  // CAN帧头信息
    uint8_t data[8];                // 数据缓冲区
    
    // 第3行：从FIFO0读取消息
    // HAL_CAN_GetRxMessage：从指定FIFO读取CAN消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) == HAL_OK) {
        // 第4行：接收计数器加1
        g_can_rx_count++;
        
        // 第5行：处理电机数据
        // rx_header.StdId：CAN标准ID
        // data：接收到的数据
        motor_process_data(rx_header.StdId, data);
    }
}
```

## 四、关键数据结构详解

### 4.1 控制模式枚举

```c
typedef enum {
    MODE_SPEED = 0,     // 速度模式：只控制电机转速
    MODE_POSITION = 1   // 位置模式：控制电机转到特定角度
} ControlMode;
```

**为什么使用枚举而不是宏定义？**
1. **类型安全**：编译器会检查类型
2. **代码可读性**：MODE_SPEED比0更有意义
3. **易于扩展**：添加新模式只需修改枚举

### 4.2 PID参数结构体

```c
typedef struct {
    float kp;           // 第1个成员：比例系数
    float ki;           // 第2个成员：积分系数
    float kd;           // 第3个成员：微分系数
    float integral;     // 第4个成员：积分项累加值
    int16_t last_error; // 第5个成员：上一次误差，用于微分计算
} PID_Param;
```

**结构体设计的考虑：**
1. **参数分组**：所有PID相关参数放在一起
2. **状态保存**：integral和last_error需要保持连续性
3. **易于调整**：通过修改结构体成员即可调整PID参数

### 4.3 电机数据结构体

```c
typedef struct {
    int16_t angle;          // 原始编码器值（0-8191）
    int16_t speed_rpm;      // 转速（RPM，有符号）
    int16_t actual_current; // 实际电流（mA，有符号）
    float angle_degree;     // 角度值（度，支持多圈）
    int32_t total_encoder;  // 总编码器计数（支持正负）
    uint8_t online;         // 在线状态（0/1）
} Motor_Data;
```

**各成员的作用：**
1. **angle**：用于计算圈内位置
2. **speed_rpm**：速度控制反馈
3. **actual_current**：监控电机负载
4. **angle_degree**：位置控制反馈
5. **total_encoder**：精确的多圈计数
6. **online**：通信状态指示

## 五、代码编写的最佳实践

### 5.1 为什么使用extern声明全局变量？

```c
// 在motor_simple.h中声明
extern int16_t target_rpm;

// 在motor_simple.c中定义
int16_t target_rpm = 500;
```

**原因：**
1. **避免重复定义**：只在.c文件中定义一次
2. **提供接口**：其他文件可以通过extern使用
3. **封装性**：隐藏变量的实际位置

### 5.2 为什么使用静态变量？

```c
static int16_t last_encoder = 0;
```

**原因：**
1. **作用域限制**：只在当前.c文件中可见
2. **保持状态**：在函数调用间保持值不变
3. **避免命名冲突**：不会与其他文件中的变量冲突



## 六、常见问题及解决方案

### 6.1 编码器溢出处理问题

```c
// 错误的做法：直接计算差值
delta = encoder - last_encoder;  // 溢出时会计算出错

// 正确的做法：检测溢出
if (last_encoder > 6000 && encoder < 2000) {
    delta = (8192 - last_encoder) + encoder;
    encoder_turns++;
}
```

**为什么阈值是6000和2000？**
1. **容错性**：避免误判正常的快速变化
2. **安全性**：确保只有真正的溢出才被检测
3. **经验值**：根据电机最大转速和采样频率确定

### 6.2 PID积分饱和问题

```c
// 必须进行积分限幅
if (vel_pid.integral > 1000) vel_pid.integral = 1000;
if (vel_pid.integral < -1000) vel_pid.integral = -1000;
```

**为什么需要积分限幅？**
1. **防止windup**：长时间误差积累导致积分项过大
2. **快速恢复**：当目标改变时，积分项不会影响响应
3. **系统稳定**：避免积分项导致系统振荡

### 6.3 电流限幅问题

```c
// 电流必须限制在合理范围
if (current > 10000) current = 10000;
if (current < -10000) current = -10000;
```

**为什么限制在±10000？**
1. **电调限制**：C620电调支持的最大值
2. **电机安全**：防止电流过大损坏电机
3. **电池保护**：防止过大的电流消耗

## 七、代码优化建议

### 7.1 可以优化的地方

```c
// 当前：使用浮点数计算
float error = target_degree - actual_degree;
float p_term = pos_pid.kp * error;

// 优化：使用定点数计算（如果CPU没有FPU）
int32_t error = (target_degree - actual_degree) * 1000;  // 放大1000倍
int32_t p_term = (pos_pid.kp * error) / 1000;
```

### 7.2 添加看门狗

```c
// 在主循环中添加
while (1) {
    IWDG_ReloadCounter();  // 喂狗
    // ... 其他代码
}
```

### 7.3 添加错误恢复

```c
void motor_error_recovery(void)
{
    if (motor_error_count > 10) {
        motor_reset();  // 电机复位
        motor_error_count = 0;
    }
}
```

## 八、总结



1. **初始化的顺序**必须正确：HAL库 → 时钟 → 外设
2. **CAN过滤器配置**要理解位操作和寄存器布局,很重要，不然can无法正常工作
3. **编码器处理**必须考虑溢出和多圈计数
4. **PID控制**要理解双环结构和参数整定
5. **调试输出**有助于实时监控系统状态


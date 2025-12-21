# 学习SPI和IMU

## 一.SPI

### 简介

1. **全双工：**

    - 可以同时发送和接收数据。

2. **主从：**

    - 一个主设备控制多个从设备。

3. **四根线：**

    - SCK（时钟线）
    - MOSI（主输出，从输入）
    - MISO（主输入，从输出）
    - CS（片选线，每个设备一根，拉低表示选中设备）

### drv_spi.cpp

#### 组成
1. **SPI_Init()**

    - 配置相应总线对应的句柄和回调函数；

2. **SPI_Send_Receive_Data()**

    - 参数包括句柄、引脚、发送长度、接收长度；
    - 配置好后调用``HAL_SPI_TransmitReceive()``或``HAL_SPI_TransmitReceive_DMA()``（前者阻塞，后者会调用回调函数）；
    - 概括：主发数据同时收到从的反馈，如果是DMA就会调用回调函数；
3. **HAL_SPI_TxRxCpltCallbcak()**

    - 对应总线发数据的时候选择了``DMA模式``才会被调用；
    - 反转CS引脚电平，使得引脚被拉高，结束从设备的选中；
    - 调用自己配置的回调函数，在那个回调函数里面进行进一步处理；

4. **TIM_SPI_PeriodElapsedCallback()**

    - 简介是：“SPI的TIM定时器中断交互回调函数”
    - ``具体实现未知``

#### 使用流程
1. **初始化：**
    - 在``Task_Init()``中调用``SPI_Init()``;
2. **定义回调函数：**
    - 在Task.cpp文件中定义回调函数，名字和初始化里面给``SPI_Init()``的参数的一样；
3. **发送和接收数据：**
    - 在需要的地方使用``SPI_Send_Receive_Data()``（这个函数里面反转引脚被注释掉了，个人觉得应该取消注释-12.18）

## 二.IMU

### 简介

- IMU：惯性测量单元 
- 包含： 
    - 三轴陀螺仪：测量X、Y、Z轴的角速度
    - 三轴加速度计：测量沿X、Y、Z轴的线性加速度

### IMU系统的工作流程

1. **初始化阶段：**
    - 配置BMI088传感器参数，启动SPI通信；
2. **运行阶段：**
    - 定时读取传感器原始数据，通过EKF算法计算出当前姿态；
3. **应用阶段：**
    - 通过接口函数获取欧拉角等姿态信息；

### **dvc_imu.h**

#### 成员
- INS_t :
    - “惯性导航系统”
    - 用于存储和处理IMU（惯性测量单元）包括姿态、角速度、加速度

- Enum_IMU_Status:

- Class_IMU :
    - 方法：
        1. ``void Init(void)``：

            - IMU配置初始化

        2. ``void TIM_Calculate_PeriodElapsedCallback(void)``：

            - 周期性姿态解算

        3. ``void TIM1msMod50_Alive_PeriodElapsedCallback(void)``:

            - 每50ms执行一次的回调函数，检测检测IMU是否正常，数据不变化就失能，变化就使能；

        4. ``float Get_Angle_Roll(void)``、``float Get_Angle_Pitch(void)``、``float Get_Angle_Yaw(void)``:

            - 获取当前yaw、pitch、roll位姿角度值；
        5. ``float Get_Rad_Roll(void)``、``float Get_Rad_Pitch(void)``、``float Get_Rad_Yaw(void)``:

            - 获取当前yaw、pitch、roll位姿弧度值；

        6. ``float Get_Accel_X(void)``、``float Get_Accel_Y(void)``、``float Get_Accel_Z(void)``:

            - 获取x、y、z轴加速度；
            
        7. ``float Get_Gyro_Roll(void)``、``float Get_Gyro_Pitch(void)``、``float Get_Gyro_Yaw(void)``:

            - 获取yaw、pitch、roll的角速度；

        8. ``Enum_IMU_Status Get_IMU_Status(void)``：

            - 获取IMU当前的状态；

## 三.出错及解决

### 问题描述

- IMU数据在Debug的时候要么是"Nan",要么是"0"

### 解决方法

1. 发现IMU_Init()里面仅仅在BMI088里有部分关于SPI初始化的代码,我在Task.cpp里面补充后,个别数据开始有值,但大部分数据还是没有值;
2. 发现dwt(用于实现微秒级延时)的数据在debug里面一直没有值,于是去检查dwt的初始化,发现imu里面没有关于dwt的初始化,所以在Task.cpp里面补充上了;
3. 由此,我意识到imu初始化不完全,于是去检查imu的代码里面有没有用到其他的外设,发现tim也没有初始化,补充上后就能正常运行了;

## ``imu配置全流程``

1. Task1ms_TIM5_Callback()里面加上以下代码;
```
    // 50秒执行一次IMU状态检测
    if (imu_counter >= 50) {
        imu_counter = 0;
        IMU.TIM1msMod50_Alive_PeriodElapsedCallback();
    }
    IMU.TIM_Calculate_PeriodElapsedCallback();
```

2. 在Task_Init()里面添加上如下代码;
```
    // DWT初始化(第一个)
    DWT_Init(168);

    ...//其他初始化

    // TIM初始化
    TIM_Init(&htim10, NULL);
    HAL_TIM_Base_Start_IT(&htim10);
    // SPI初始化
    SPI_Init(&hspi1, NULL);
    // IMU初始化
    IMU.Init();
```

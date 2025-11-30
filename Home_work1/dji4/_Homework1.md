###can通信基础知识
###一、
**CAN_TxHeaderTypeDef**不需要自己定义！它是 STM32 HAL 库自带的 标准结构体类型，专门用于存储 CAN 发送报文的 “头部配置信息”。
通过 #include "main.h"（main.h 中会包含 stm32xxx_hal.h，进而包含所有外设库头文件）间接引入工程，因此直接声明变量时，编译器能识别到该类型，不会报错。
**typedef struct**
{
  uint32_t StdId;     //11 位标准 ID :0 ~ 0x7FF
  uint32_t ExtId;     //29 位扩展 ID
  uint32_t IDE;       //ID 类型标识 区分用
  uint32_t RTR;       //帧类型标识（区分数据帧 / 远程帧）
  uint32_t DLC;       //数据长度码
  uint32_t Timestamp;   //接收时间戳（暂时用不到）
  uint32_t FilterMatchIndex; //匹配的滤波器索引
} **CAN_RxHeaderTypeDef;**
**补充：**
1.*IDE*：ID 类型标识（区分标准 / 扩展 ID）
核心作用：告诉用户当前接收到的报文是 “标准 ID” 还是 “扩展 ID”，避免混淆 StdId 和 ExtId 的值。
取值来源：HAL 库定义的*枚举类型* CAN_identifier_type（无需用户定义，直接使用），仅两个有效取值：
CAN_ID_STD（值为 0）：标准 ID 类型（此时 StdId 有效，ExtId 无效）；
CAN_ID_EXT（值为 4）：扩展 ID 类型（此时 ExtId 有效，StdId 无效）。
2.*RTR*：帧类型标识（区分数据帧 / 远程帧）
核心作用：告诉用户当前接收到的报文是 “数据帧”（带实际数据）还是 “远程帧”（仅请求数据，无实际数据）。
取值来源：*HAL 库枚举*CAN_remote_transmission_request，仅两个有效取值：
CAN_RTR_DATA（值为 0）：数据帧（最常用，携带 0-8 字节数据，比如 RM 机器人中底盘发送的速度数据、云台发送的角度数据）；
CAN_RTR_REMOTE（值为 2）：远程帧（无数据，仅用于 “请求其他设备发送数据”，例：主控发送远程帧给传感器，请求其上传当前温度）。

####注意：
1.与 CAN_TxHeaderTypeDef 对应（基本一致），HAL 库还提供了 CAN_RxHeaderTypeDef（接收头部结构体），用于存储接收到的 CAN 报文的头部信息
2.**接收方的 StdId 对应发送方的 TxHeader.StdId，IDE/RTR/DLC 也一一对应，通信双方必须一致才能正确解析；**
3.无效成员的处理：若使用标准 ID，无需关注 ExtId；若未启用时间触发模式，无需关注 Timestamp，避免使用无效值导致错误。

###二、
**void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)**
CAN 接收中断的核心回调函数，专门用于处理 “CAN1 的 RX_FIFO0 缓冲区有新报文挂起” 的中断事件。
{
  **if(hcan->Instance == CAN1)**
}
区分中断来源 多个can 区分can1 can2。

###三、
**HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Can_Rx_Data);**
 HAL 库提供的 CAN 接收数据核心 API，作用是从 CAN 的 FIFO 缓冲区中读取接收到的报文，自动填充 “报文头部信息” 和 “数据内容”。
 *hcan*	CAN 外设句柄指针（指定从哪个 CAN 外设读取，这里是 CAN1）
*CAN_RX_FIFO0*	指定读取哪个 FIFO 缓冲区（CAN 有 2 个接收 FIFO：FIFO0 和 FIFO1）
*&RxHeader*	接收报文头部结构体指针（存储报文的 ID、帧类型、数据长度等元信息）	
*Can_Rx_Data*	接收数据缓存数组（存储报文的实际数据内容，最大 8 字节）	

**API 执行逻辑：**
函数内部会从 CAN1 的 RX_FIFO0 缓冲区中，读取最新接收到的报文；
自动将报文的 “头部信息”（如发送方 ID、数据长度 DLC、帧类型 RTR 等）填充到 RxHeader 结构体中；
自动将报文的 “实际数据”（最多 8 字节）填充到 Can_Rx_Data 数组中；
读取完成后，RX_FIFO0 缓冲区会自动清空该报文（为接收下一条报文做准备）。
**HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Can_Rx_Data) != HAL_OK**
*!= HAL_OK*：判断函数返回值 “不等于 HAL_OK”—— 即函数执行失败（可能是 HAL_ERROR/HAL_BUSY/HAL_TIMEOUT 中的任意一种）；
整体逻辑：“如果读取 CAN 接收数据失败，则执行后续代码（如错误处理）”。

###四、
  //  步骤1：配置CAN滤波器 
  CAN_FilterTypeDef CAN_FilterConfig;
  CAN_FilterConfig.FilterBank = 0;                  // 滤波器组0（CAN1用0-13）
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码匹配模式
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位滤波器
  CAN_FilterConfig.FilterIdHigh = 0x0000;           // 滤波器ID高16位（0=接收所有）
  CAN_FilterConfig.FilterIdLow = 0x0000;            // 滤波器ID低16位
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;       // 掩码高16位（0=不限制）
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;        // 掩码低16位
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 匹配报文存入FIFO0
  CAN_FilterConfig.FilterActivation = ENABLE;       // 启用滤波器
  CAN_FilterConfig.SlaveStartFilterBank = 14;       // CAN2滤波器起始组（固定14）
  HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfig);   // 应用滤波器配置

  //  步骤2：启动CAN + 激活接收中断
  HAL_CAN_Start(&hcan); // 启动CAN1外设（必须启动才能收发数据）
  // 激活RX_FIFO0消息挂起中断（触发后调用回调函数）
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  //  步骤3：配置CAN发送头部
  TxHeader.StdId = 0x123;                // 发送标准ID=0x123
  TxHeader.RTR = CAN_RTR_DATA;           // 数据帧（非远程帧）
  TxHeader.IDE = CAN_ID_STD;             // 标准ID类型（11位）
  TxHeader.DLC = 8;                      // 数据长度8字节
  TxHeader.TransmitGlobalTime = DISABLE; // 禁用发送时间戳
理解：TxHeader 是 CAN 发送报文的 “身份信息头”，相当于给数据加了 “快递单”—— 接收方通过这些信息判断 “这是谁发的、是什么类型的报文、要读多少数据”。**接收方的滤波器必须和这些信息匹配**，才能正确接收。
  //  步骤4：发送一次CAN数据
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, Can_Tx_Data, &TxMailbox);
  HAL_CAN_AddTxMessage 将 “发送头部 + 数据” 写入 CAN 发送邮箱
  *&TxMailbox*	输出参数（记录数据存入哪个发送邮箱）	定义了 uint32_t TxMailbox（CAN 有 3 个发送邮箱，值为 0/1/2）。
**发送逻辑细节**
函数执行时，会先检查 CAN 的 3 个发送邮箱是否有空闲；
若有空闲，将 TxHeader 和 Can_Tx_Data 写入该邮箱，TxMailbox **会被赋值**为邮箱编号（0/1/2）；
硬件会自动检测 CAN 总线是否空闲，空闲时将邮箱中的数据发送到总线；
发送完成后，该邮箱会自动变为 “空闲”，可接收下一次发送请求；
若 3 个邮箱都忙，函数会返回 HAL_BUSY（发送失败），你可以后续重试。
**注意**发送成功不代表接收方一定收到：发送成功仅表示 “数据已写入邮箱并由硬件发送到总线”，是否被接收方接收，取决于接收方的滤波器配置（是否匹配 StdId=0x123）和总线连接（如终端电阻是否正常）。

###dji3508 电机
#####1.**电机ID VS 电调ID**
对于M3508电机，两者完全等同，改变电调ID会直接改变电机ID
    查看M3508电机ID的方法：通过C620电调LED指示灯
######2.**ID与can通信的关系**
电调ID 1 ->can标识符 0x201 ->对应电机 M3508-1
电调ID 2 ->can标识符 0x202 ->对应电机 M3508-2
同一CAN总线上的电调ID必须唯一。

####围绕 “CAN 通信 ID 定义、电机反馈数据存储、控制指令发送、反馈数据读取” 
**CAN 通信 ID 定义**
**1.** 硬件总线绑定宏
#define CHASSIS_CAN hcan1     // 3508 电机绑定到 CAN1 总线（但stm32f103c8t6只有can，不分can1和can2）
作用：明确 3508 电机的通信总线是 hcan1（STM32 的 CAN1 外设），所有 3508 相关的 CAN 收发操作都通过 CAN1 完成；
关联：和 main.c 中 MX_CAN1_Init() 初始化的 CAN1 外设对应，确保硬件通路一致。
**2.** 3508 专属 CAN ID 枚举
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,  // 3508 电机批量控制 ID（同时控制4个电机）
    CAN_3508_M1_ID = 0x201,      // 3508 电机1 单独控制/反馈 ID
    CAN_3508_M2_ID = 0x202,      // 3508 电机2 单独控制/反馈 ID
    CAN_3508_M3_ID = 0x203,      // 3508 电机3 单独控制/反馈 ID
    CAN_3508_M4_ID = 0x204,      // 3508 电机4 单独控制/反馈 ID
    // ... 其他电机ID（暂时忽略，仅关注M3508）
} can_msg_id_e;
反馈 ID 与控制 ID 一致：3508 电机执行指令后，会通过自身的控制 ID（如 0x201）向 CAN 总线回传状态数据（转速、位置等）。

**电机反馈数据存储**
**3.**3508 电机反馈数据结构体
typedef struct
{
    uint16_t ecd;             // 3508 电机编码器原始值（16位，范围0~65535）
    int16_t speed_rpm;        // 3508 电机实际转速（单位：转/分，可正可负，对应正反转）
    int16_t given_current;    // 3508 电机当前接收的控制电流（单位：mA 级，对应指令电流）
    uint8_t temperate;        // 3508 电机绕组温度（单位：℃，范围0~127）
    int16_t last_ecd;         // 上一次读取的编码器值（用于处理编码器溢出，计算总圈数）
} motor_measure_t;
关键成员详解（3508）：
ecd：3508 内置 16 位磁编码器的原始值，1 圈对应 8192 个脉冲（DJI 3508 标准参数），需结合 last_ecd :计算电机实际转动圈数（避免 16 位数值溢出）；
speed_rpm：电机实时转速，正负数对应正反转（如 3000 表示每分钟 3000 转，-1500 表示反向每分钟 1500 转）；
given_current：电机当前执行的 “实际控制电流”（与用户发送的指令电流一致，可用于校验指令是否被正确接收）；
temperate：电机温度，超过 85℃ 需注意散热（避免电机过热保护,下电吧）。

**控制指令发送**
**4.**单个 / 批量控制 3508 电机：CAN_cmd_chassis
**3508 是电流控制型电机，电流决定转矩 / 转速**
/**
  * @brief          发送 3508 电机控制电流（0x201~0x204）
  * @param[in]      motor1: (0x201) 3508 电机1 控制电流，范围 [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 电机2 控制电流，范围 [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 电机3 控制电流，范围 [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 电机4 控制电流，范围 [-16384,16384] 
  * @retval         none
  */
**extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);**
核心功能：通过 CAN1 总线，向 4 个 3508 电机（ID 0x201~0x204）分别发送电流控制指令，支持 “单独控制某个电机” 或 “同时控制 4 个电机”；
电流参数关键说明（3508 专属）：
数值范围：[-16384, 16384]，对应实际电流 [-20A, 20A]（编码比例：16384 = 20A → 1 数字量 = 20A / 16384 ≈ 0.00122A = 1.22mA）；
正负意义：正数对应电机正转，负数对应反转（方向由电机接线和软件定义一致）；
实际应用：比如 CAN_cmd_chassis(4096, 4096, 4096, 4096) 表示 4 个 3508 电机都输出 5A 电流（4096 × 1.22mA ≈ 5A），带动底盘前进。

**反馈数据读取**
/**
  * @brief          获取底盘 3508 电机的反馈数据
  * @param[in]      i: 电机编号，范围 [0,3]（0=电机1，1=电机2，2=电机3，3=电机4）
  * @retval         电机反馈数据结构体指针（const 修饰，防止误修改）
  */
**extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);**

**3508 电机的工作流程**如下：
初始化：STM32 初始化 CAN1（MX_CAN1_Init()）→ 配置 CAN 滤波器（can_filter_init()，允许接收 0x201~0x204 的反馈报文）→ 启动 CAN1 并激活中断；
发送控制指令：main.c 中调用 CAN_cmd_chassis(4000, 4000, 4000, 4000)，通过 CAN1 向 4 个 3508 电机发送～4.88A 的电流指令；
电机执行与反馈：3508 电机接收指令后，输出对应转矩 / 转速，同时通过 CAN1 向 STM32 回传自身状态（ecd、speed_rpm 等）；
接收与解析：STM32 触发 CAN1 接收中断，中断回调函数中解析反馈报文，将数据填充到 motor_measure_t 结构体；
读取反馈数据：用户通过 get_chassis_motor_measure_point(0) 等函数读取电机状态，实现闭环控制（如根据转速调整电流指令，让电机稳定在目标转速）。
###pid
（数学上和代码上有转换）
P：快速响应当前误差；
i：消除积累静差；
d：抑制未来震荡
调试时先p再i再d

###Tips：
1.**按位与的使用避免数组越界**
. 函数体：return &motor_chassis[(i & 0x03)];
i & 0x03：电机编号的 “安全门禁”
作用：强制把参数 i 的值限制在 0~3 之间，防止传错值导致程序崩溃；
原理：**0x03 是十六进制，转成二进制是 00000011。任何数字和 00000011 做 “与运算（&）”，结果只能是 0~3（二进制 00~11）；**
eg：
传对值：i=2 → 2 & 0x03 = 2（正常返回电机 3）；
传错值：i=4 → 4 & 0x03 = 0（自动变成 0，返回电机 1，不会崩溃）；
传负数：i=255 → 255 & 0x03 = 3（自动变成 3，返回电机 4）；
通俗讲：这是个 “容错设计”—— 哪怕你不小心传了错误的编号（比如 4、100），函数也会自动修正为 0~3，避免数组越界（访问 motor_chassis[4] 及以上，导致程序卡死）。
2.**case 穿透（无 break 串联）**
C 语言中，switch-case 的规则是：
当某个 case 的条件匹配后，会从该 case 开始往下执行，直到遇到 break 或 switch 大括号结束 —— 如果某个 case 后面没有 break，程序会「穿透」到下一个 case 继续执行，不会中断。
代码里：
四个 case 后面都没有加 break，所以它们是「串联在一起」的；
无论 rx_header.StdId 匹配的是 CAN_3508_M1_ID、M2、M3 还是 M4，最终都会执行到 case 后面大括号 { } 里的代码（包括 get_motor_measure）。
3.**#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))**              限制范围

###冒出的问题以及解决办法：
1.问题一：我只控制电机 1（M1），能不能往 0x201 发控制指令？                —— 不能
DJI 给 3508 的 CAN 通信分配了*控制 ID和反馈 ID两套独立体系*，各司其职，不能交叉使用：
简单地说，
**0x200 是「电机听指令的通道」，只负责接收；**
**0x201~0x204 是「电机说话的通道」，只负责发送；**
电机固件不会解析其他 ID（比如 0x201、0x300）的控制帧，哪怕你往 0x201 发控制电流，电机也会直接丢弃，完全不响应。
2.问题二：这个pid输出值是转速吗，驱动3508电机用的不是电流值吗？            --有解决办法
PID 的本质是「比例放大 + 积分消除静差 + 微分抑制震荡」，这个 “放大 / 消除 / 抑制” 的过程，就是把「转速差」转换成「电流指令」的过程。
简单地说，
**通过调整pid的系数值，再看两条曲线的波形图就可以成功把转速差转换成电流。**
3.问题三：  HAL_CAN_Start(&hcan);//启动can外设
    HAL_CAN_ConfigFilter(&hcan, &can_filter_st);//配置过滤器
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//开启接收中断
    这三者有没有先后顺序？
4.问题四：一开始只能开环控制电机 无法进入接收中断函数 收不到反馈数据啊（也是深刻意识到了之前不会调试有点说不过去了，学会了全局变量和局部变量在watch窗口的能否被看见情况，cannot evaluate...） 
         和队友讨论 向队友求助，对比了cubemx里的配置之后 决定把can那四个全开了（虽然不知道多开有啥用） 以及把滤波器参数弄成来者不拒（目前就一个can）
         这么一改 奇迹般地能收到反馈值了（还是想知道为什么）
5.问题五：ozone 打不开文件。。。
          感谢两位学长相助
6.问题六：在B站学习了相关理论，可是没法直接对应到代码上，先做了stm32大作业的can通信题
         再学习了RM官网的can通信例程，了解代码基本框架，专注dji3508电机部分，加入pid
7.问题七：看完了例程以及学习文档有很多困惑，组长培训＋ai解答 解决了疑惑 学会了更多
          eg：硬件连接，共地要求stm32GND连CAN收发器的GND，CAN收发器的GND连C620电调GND（迫于连线方式，间接共地也行）；获取C620通过指示灯变化提供的信息；怎么改变电机ID；控制 ID和反馈 ID是两套独立体系；C620电调对「接收控制指令的CAN ID（0x200）」的过滤规则是 出厂固件固化的硬件级逻辑，只需要确保STM32发送的控制帧ID是0x200即可，无需额外配置；C620发送端确实无过滤：C620电调无任何发送过滤，只要上电且电机运转，就会固定发送反馈帧（电机1=0x201，电机2=0x202，电机3=0x203，电机4=0x204）
发送频率固定为1KHz（每1ms一次），持续广播自身状态，不受STM32控制，也无法通过软件修改
STM32接收端必须配置过滤器：
STM32的CAN控制器默认不接收任何帧，必须通过软件配置过滤器来决定接收哪些ID的帧。要分清32给电机发指令和电机反馈信息给32两个情况下的发送和接收端分别是。
###总结：
这周创建了github账号，学了基本git技能，简单配置了vscode，学会用ozone展现波形图，keil调试小知识
can通信，dji电机3508，简单pid
###体会：
发现大家都很全身心的在投入，有被激励到。
赶任务的时候是真急，完成任务的时候是真爽。
归去时是万事开头难，归来时便已是轻舟过万重山，继续加油吧。

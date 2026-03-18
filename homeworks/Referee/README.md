# 裁判系统 学习记录

## 裁判系统介绍

> 裁判系统是集成计算、通信、控制于一体的针对机器人比赛的电子判罚系统。裁判系统整体包含：安装于机器人上的机载端，以及安装在PC物理机上的服务器和客户端软件。
>
> 机载端包含主控模块、装甲模块、测速模块、场地交互模块、相机图传模块、定位模块等，各模块组合成的系统可以感知机器人对抗过程中的伤害、发射弹丸的速度与频率、同时可以将机器人的第一视角画面传回客户端等。
>
> 服务器和客户端软件可以实时查看所有机器人的实时状态，根据比赛规则自动判定比赛胜负，同时可以通过服务器和客户端软件向机器人发送控制指令完成相应的操作。[^1]

简而言之，裁判系统是一个将比赛场上的机器人与实际的比赛状态相联系起来的系统。裁判系统通过向机器人发送数据和指令，实现对比赛进程的控制。

裁判系统由三部分组成，分别为服务器、客户端、和位于机器人上的机载端。同时裁判系统有许多模块，可以辅助控制和传递信息。

## 裁判系统的通信协议

裁判系统主要采用串口协议进行通信。常规链路的波特率为 115200，图传链路的波特率为 921600，8 位数据位，1 位停止位，无硬件流控，无校验位。

### 通信协议格式

<table>
	<tr>
		<th>frame_header</th>
		<th>cmd_id</th>
		<th>data</th>
		<th>frame_tail</th>
	</tr>
	<tr>
		<td>5-byte</td>
		<td>2-byte</td>
		<td>n-byte</td>
		<td>2-byte，CRC16，整包校验</td>
	</tr>
</table>

#### 帧头

<table>
	<tr>
		<th>域</th>
		<th>偏移位置</th>
		<th>大小（字节）</th>
		<th>详细阐述</th>
	</tr>
	<tr>
		<td>SOF</td>
		<td>0</td>
		<td>1</td>
		<td>数据帧起始字节，固定值为 0xA5</td>
	</tr>
	<tr>
		<td>data_length</td>
		<td>1</td>
		<td>2</td>
		<td>数据帧中 data 的长度</td>
	</tr>
	<tr>
		<td>seq</td>
		<td>3</td>
		<td>1</td>
		<td>包序号</td>
	</tr>
	<tr>
		<td>CRC8</td>
		<td>4</td>
		<td>1</td>
		<td>帧头 CRC8 校验</td>
	</tr>
</table>

#### 命令码(`cmd_id`)和数据(`data`)

命令码长度为2个字节，具体的命令码和所属的数据链路及对应的数据格式详见官方通信协议文件[^2]进行使用。

### CRC校验

裁判系统中使用到的CRC为CRC8和CRC16。其中CRC8的多项式为

$$
x^8+x^5+x^4+1
$$

CRC的具体校验函数详见官方通信协议文件[^2]的附录一。实际使用一般为查表，而非现场计算。

### UI绘制

UI绘制部分需要机器人发送而非接收数据帧。需要发送的数据帧的`cmd_id`为`0x0301`，同时，发送出的数据帧的格式也比较特殊，包含了一个子内容数据帧，即下表中的内容数据段。

<table>
    <tr>
        <th>字节偏移量</th>
        <th>大小</th>
        <th>说明</th>
        <th>备注</th>
    </tr>
    <tr>
        <td>0</td>
        <td>2</td>
        <td>子内容 ID</td>
        <td>需为开放的子内容 ID</td>
    </tr>
    <tr>
        <td>2</td>
        <td>2</td>
        <td>发送者 ID</td>
        <td>需与自身 ID 匹配，ID 编号详见附录</td>
    </tr>
    <tr>
        <td>4</td>
        <td>2</td>
        <td>接收者 ID</td>
        <td>
            <ul>
                <li>仅限己方通信</li>
                <li>需为规则允许的多机通讯接收者</li>
                <li>若接收者为选手端，则仅可发送至发送者对应的选手端</li>
                <li>ID 编号详见附录</li>
            </ul>
        </td>
    </tr>
    <tr>
        <td>6</td>
        <td>x</td>
        <td>内容数据段</td>
        <td>x 最大为 112</td>
    </tr>
</table>

#### 子内容数据段

子内容的ID在主内容数据帧中已经定义了。子内容数据帧许多种类，包括对图层的删除、单个或多个图形的绘制、图形或字符的增删改等。具体格式详见官方通信协议文件[^2]的表1-24至表1-31。

## 具体代码实现

### 接受事件代码

```c
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    Referee.UART_RxCpltCallback(Buffer, Length);
}
```

然后在`Task_Init`函数中进行事件的注册。

```
UART_Init(&huart6, Referee_UART6_Callback, 128);
```

### 具体解包细节

裁判系统具体的解包过程在`dvc_referee.cpp`文件中的`Class_Referee::Data_Process`函数中。

#### 寻找数据包的开头

遍历UART接收缓存字节数组，查找帧头`0xA5`，记录当前的数组索引。

```c++
buffer_index = 0;
buffer_index_max = UART_Manage_Object->Rx_Buffer_Length;
// 遍历整个接收缓冲区寻找帧头
while (buffer_index < buffer_index_max)
{
    // 通过校验和帧头
    if (UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index)] == 0xA5)
    {
        ...
    }
    buffer_index++;
}
```

这之中有一个名为`Get_Circle_Index`的函数，作用如其名称所示。遍历查找帧头和循环索引大概是为了DMA循环模式而准备的。

#### 数据处理过程

这部分主要是把需要用到的`cmd_id`与`data_length`以及`data`部分解析出来，同时对帧头以及整个数据进行CRC校验。

```c++
// 数据处理过程
cmd_id = (UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 6)]) & 0xff;
cmd_id = (cmd_id << 8) | UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 5)];
data_length = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 2)] & 0xff;
data_length = (data_length << 8) | UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 1)];
Math_Constrain(&data_length, (uint16_t)0, (uint16_t)(128)); // 限制数据段最大长度
Enum_Referee_Command_ID CMD_ID = (Enum_Referee_Command_ID)cmd_id;

uint8_t *data_temp = new uint8_t[5];
uint8_t *sum_data = new uint8_t[data_length + 9];
for (int i = 0; i < 5; i++)
{
    data_temp[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + i)];
}
if (Verify_CRC8_Check_Sum(data_temp, 5) == 1) // 校验帧头
{
    for (int i = 0; i < data_length + 9; i++)
    {
        sum_data[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + i)];
    }
    if (Verify_CRC16_Check_Sum(sum_data, data_length + 9) == 1) // 校验整个帧
    {
        ...
    }
}
```

#### 根据命令码解析数据

```c++
switch (CMD_ID)
{
    case ...:
        break;
    ...
}
```

由于裁判系统的数据包格式经常变动，我们需要在使用前检查使用到的`cmd_id`及其值是否正确，具体处理过程中的数据包结构体的格式是否正确。这些部分都定义在`dvc_referee.h`文件中，需要参考官方通信协议文件[^2]的1.2节。

### UI绘制

#### UI数据的打包和发送

仓库中的代码其实是将要发送的子内容数据封装起来，发送时视为正常数据发送，所以要先了解数据帧的发送。其实就是将数据帧的帧头、`cmd_id`的`0x0301`以及数据帧中的子内容ID、发送者ID以及接受者ID配置好，然后用子内容数据写入剩余部分，最后计算CRC校验值即可。发送时只需要使用`HAL_UART_Transmit`函数即可。

```c++
/**
 * @brief 裁判系统数据打包
 *
 */
template <typename T>
void Class_Referee::Referee_UI_Packed_Data(T *__data)
{
    uint16_t frame_length, data_len, cmd_id;

    cmd_id = 0x0301;                                               // 子内容ID
    data_len = sizeof(T);                                          // 字符操作数据长度
    frame_length = frameheader_len + cmd_len + data_len + crc_len; // 数据帧长度

    memset(UART_Manage_Object->Tx_Buffer, 0, frame_length); // 存储数据的数组清零

    /*****帧头打包*****/
    UART_Manage_Object->Tx_Buffer[0] = Frame_Header;                       // 数据帧起始字节
    memcpy(&UART_Manage_Object->Tx_Buffer[1], (uint8_t *)&data_len, 2);    // 数据帧中data的长度
    UART_Manage_Object->Tx_Buffer[3] = seq;                                // 包序号
    Append_CRC8_Check_Sum(UART_Manage_Object->Tx_Buffer, frameheader_len); // 帧头校验CRC8

    /*****命令码打包*****/
    memcpy(&UART_Manage_Object->Tx_Buffer[frameheader_len], (uint8_t *)&cmd_id, cmd_len);

    /*****数据打包*****/
    memcpy(&UART_Manage_Object->Tx_Buffer[frameheader_len + cmd_len], __data, sizeof(T));
    Append_CRC16_Check_Sum(UART_Manage_Object->Tx_Buffer, frame_length); // 一帧数据校验CRC16

    UART_Manage_Object->Tx_Length = frame_length;

    seq++;
}

// 发送数据
HAL_UART_Transmit(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length, 10); // 阻塞发送
```

#### 子内容数据的封装和解析

接下来以绘制字符图形为例，解释子内容数据的封装。

首先是字符图形结构体的定义。

```c++
/**
 * @brief 裁判系统发送的数据, 0x0301画字符图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_String
{
    uint16_t Header = 0x0110;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_String;
    uint8_t String[30];
} __attribute__((packed));
```

具体各部分内容的含义由于过长，详见官方通信协议文件[^2]。

之后是如何将这个结构体变成可以发送的字节数组。具体来讲就是按照格式要求，向相应内容填写合适的数据。

```c++
/**
 * @brief 绘制字符串
 *
 */
void Class_Referee::Referee_UI_Draw_String(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint32_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Font_Size, uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, char *__String, uint32_t __String_Length, Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_String.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_String.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    memcpy(Interaction_Graphic_String.String, __String, __String_Length * sizeof(uint8_t));
    Interaction_Graphic_String.Graphic_String.String.Serial = __Serial;
    Interaction_Graphic_String.Graphic_String.String.Index[0] = __Index;
    Interaction_Graphic_String.Graphic_String.String.Operation_Enum = __Operate_Type;
    Interaction_Graphic_String.Graphic_String.String.Type_Enum = 7;
    Interaction_Graphic_String.Graphic_String.String.Color_Enum = __Color;
    Interaction_Graphic_String.Graphic_String.String.Font_Size = __Font_Size;
    Interaction_Graphic_String.Graphic_String.String.Line_Width = __Line_Width;
    Interaction_Graphic_String.Graphic_String.String.Start_X = __Start_X;
    Interaction_Graphic_String.Graphic_String.String.Start_Y = __Start_Y;
    Interaction_Graphic_String.Graphic_String.String.Length = __String_Length;
}
```

在要绘制字符图形时，就先调用`Referee_UI_Draw_String`函数，将字符的格式与文本封装到数据包中，然后调用`Referee_UI_Packed_Data`将字符图形结构体封入要发送的UART字节数据中，最后调用`HAL_UART_Transmit`进行数据的发送。

```c++
Referee_UI_Draw_String(Get_ID(), Referee_UI_Zero, 0, 0x00, 0, 20, 2, 500, 500, "Chassis", (sizeof("chassis") - 1), Referee_UI_ADD); // 配置字符信息
Referee_UI_Packed_Data(&Interaction_Graphic_String); // 打包字符数据
HAL_UART_Transmit(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length, 10); // 阻塞发送
```

其他类型的图形也同理。

## 总结

本次学习了裁判系统，裁判系统是`Robomaster`比赛进行的基础，需要深刻理解。同时本次还学习了CRC校验，CRC校验是一种高效率的校验算法，在各个领域，例如CAN、USB等，都有广泛的应用。


---

## 参考文献

[^1]:https://bbs.robomaster.com/wiki/20204847/804679?source=7

[^2]:https://bbs.robomaster.com/wiki/20204847/811363?source=7


# STM32遥控控制帧解析学习日志

## 核心功能模块

### 1. 控制帧结构定义
在`control_frame.h`中定义了遥控器控制帧的数据结构：
- 4个通道数据(ch0-ch3)：每个通道11位，范围364-1684
- 2个开关(s1,s2)：每个2位，表示开关状态
- 鼠标数据(mouse_x,mouse_y,mouse_z)：有符号16位整数
- 鼠标按键(mouse_left,mouse_right)：各1位
- 按键(key)：16位按键状态
- 保留字段(reserve)：16位备用

### 2. 数据打包与解包
- `ControlFrame_Pack`: 将控制帧结构体打包成18字节数据流
- `ControlFrame_Decode`: 将18字节数据流解码为控制帧结构体
- `ControlFrame_ReceiveAndDecode`: 结合UART接收与解码功能

## 学习要点总结

### 协议解析技巧
- 位操作在协议解析中的重要性
- 数据对齐和字节序处理
- 错误检查和边界条件处理

### 嵌入式开发实践
- STM32 HAL库的使用方法
- UART中断和DMA传输配置

### 调试经验
- 使用调试数组观察中间数据

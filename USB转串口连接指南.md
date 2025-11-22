# USB转串口连接使用指南

## 📋 概述

如果您的上位机（PC）通过USB转串口模块连接到STM32，请按照本指南进行配置。

## 🔌 硬件连接

### USB转串口模块连接

```
USB转串口模块          STM32
─────────────────────────────────
VCC (5V)      →      不连接（STM32独立供电）
GND           →      GND（必须共地）
TX            →      PB11 (USART3_RX)
RX            →      PB10 (USART3_TX)
```

**⚠️ 重要提示：**
- **不要连接VCC**：STM32应该独立供电，USB转串口模块只提供信号连接
- **必须共地**：GND必须连接，否则无法正常通信
- **TX/RX交叉连接**：模块的TX接STM32的RX，模块的RX接STM32的TX

### 常见USB转串口模块

| 模块型号 | 驱动芯片 | Windows驱动 | Linux支持 |
|---------|---------|------------|----------|
| CH340 | CH340G | 需要安装CH340驱动 | 内核自带 |
| CP2102 | CP2102 | 需要安装CP2102驱动 | 内核自带 |
| FT232 | FT232RL | 需要安装FTDI驱动 | 内核自带 |
| PL2303 | PL2303 | 需要安装PL2303驱动 | 内核自带 |

## 💻 Windows系统配置

### 1. 安装USB转串口驱动

#### CH340驱动安装
1. 下载CH340驱动（可从官网或搜索引擎下载）
2. 插入USB转串口模块
3. 打开设备管理器，找到"端口(COM和LPT)"
4. 如果看到黄色感叹号，右键选择"更新驱动程序"
5. 选择驱动文件所在目录进行安装

#### CP2102驱动安装
1. 从Silicon Labs官网下载CP210x驱动
2. 安装驱动后重启电脑
3. 插入USB转串口模块，系统会自动识别

### 2. 查看串口号

1. 插入USB转串口模块
2. 打开"设备管理器"（Win+X → 设备管理器）
3. 展开"端口(COM和LPT)"
4. 找到您的USB转串口设备，例如：
   - `USB-SERIAL CH340 (COM3)`
   - `Silicon Labs CP210x USB to UART Bridge (COM4)`
   - `Prolific USB-to-Serial Comm Port (COM5)`

**记录下COM号**（如COM3、COM4等）

### 3. 运行上位机程序

#### 方式一：使用C++上位机程序

```bash
# 编译程序
g++ -std=c++17 car_controller.cpp -o car_controller.exe

# 运行程序
car_controller.exe
```

程序启动后会提示：
```
请输入串口号 (Windows默认: COM3, Linux默认: /dev/ttyUSB0): 
```

**输入您查到的COM号**，例如：`COM4`

#### 方式二：使用Python上位机程序

```bash
# 安装依赖
pip install pyserial

# 运行程序
python car_controller.py
```

程序启动后会提示输入串口号，输入您的COM号即可。

### 4. 修改默认串口号（可选）

如果您的串口号固定，可以修改代码中的默认值：

**C++版本 (car_controller.cpp)**：
```cpp
// 第698行附近
if (port.empty()) {
#ifdef _WIN32
    port = "COM4";  // 修改为您的COM号
#else
    port = "/dev/ttyUSB0";
#endif
}
```

**Python版本 (car_controller.py)**：
```python
# 修改默认串口号
port = input("请输入串口号 (默认: COM4): ").strip()
if not port:
    port = "COM4"  # 修改为您的COM号
```

## 🐧 Linux系统配置

### 1. 查看串口设备

插入USB转串口模块后，运行：

```bash
# 查看所有串口设备
ls -l /dev/ttyUSB* /dev/ttyACM*

# 或者使用dmesg查看
dmesg | grep tty
```

常见的设备名称：
- `/dev/ttyUSB0` - CH340、CP2102等
- `/dev/ttyACM0` - CDC-ACM设备

### 2. 设置串口权限

```bash
# 添加当前用户到dialout组
sudo usermod -a -G dialout $USER

# 或者直接设置权限（临时）
sudo chmod 666 /dev/ttyUSB0

# 重新登录使组权限生效
```

### 3. 运行上位机程序

```bash
# C++版本
g++ -std=c++17 car_controller.cpp -o car_controller
./car_controller

# Python版本
python car_controller.py
```

输入串口设备路径，例如：`/dev/ttyUSB0`

## ⚙️ 串口参数配置

### 下位机配置（STM32）

串口参数已在代码中配置：
- **波特率**: 115200
- **数据位**: 8
- **停止位**: 1
- **校验位**: None
- **流控**: None

位置：`Core/Src/usart.c` 第56行

### 上位机配置

上位机程序会自动配置相同的参数，无需手动设置。

## 🔍 故障排查

### 问题1：找不到串口设备

**Windows:**
- 检查设备管理器中是否有黄色感叹号
- 重新安装USB转串口驱动
- 尝试更换USB接口
- 检查USB线是否支持数据传输（有些线只能充电）

**Linux:**
- 运行 `lsusb` 查看USB设备是否识别
- 检查 `/dev/ttyUSB*` 是否存在
- 确认用户权限（需要dialout组）

### 问题2：无法打开串口

**可能原因：**
1. 串口被其他程序占用
   - 关闭串口调试助手、Arduino IDE等
   - 检查是否有其他程序在使用该串口

2. 串口号错误
   - 重新查看设备管理器确认COM号
   - 尝试不同的COM号

3. 权限不足（Linux）
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

### 问题3：通信无响应

**检查清单：**
1. ✅ **GND是否连接**：必须共地
2. ✅ **TX/RX是否交叉**：模块TX接STM32 RX，模块RX接STM32 TX
3. ✅ **波特率是否匹配**：确认是115200
4. ✅ **STM32是否上电**：检查电源指示灯
5. ✅ **串口线是否正常**：尝试用串口调试助手测试

### 问题4：数据乱码

**可能原因：**
- 波特率不匹配：检查是否为115200
- 数据位/停止位/校验位不匹配：确认是8N1
- 串口线质量问题：更换质量好的USB转串口模块

## 🧪 测试串口连接

### 使用串口调试助手测试

1. 打开串口调试助手（如PuTTY、串口助手等）
2. 配置参数：
   - 波特率：115200
   - 数据位：8
   - 停止位：1
   - 校验位：None
3. 打开串口
4. 发送测试命令：
   ```
   CMD:STATUS
   ```
5. 应该收到STM32的响应

### 使用命令行测试（Linux）

```bash
# 发送命令
echo "CMD:STATUS" > /dev/ttyUSB0

# 读取响应（需要另一个终端）
cat /dev/ttyUSB0
```

## 📝 使用示例

### 完整连接流程

1. **硬件连接**
   ```
   USB转串口模块 → STM32
   GND → GND
   TX → PB11 (USART3_RX)
   RX → PB10 (USART3_TX)
   ```

2. **安装驱动**（Windows需要）
   - 下载对应驱动并安装
   - 确认设备管理器中显示COM号

3. **运行上位机程序**
   ```bash
   # Windows
   car_controller.exe
   输入: COM4  # 您的COM号
   
   # Linux
   ./car_controller
   输入: /dev/ttyUSB0
   ```

4. **切换到上位机模式**
   - 程序启动后会自动发送 `CMD:MODE,1`
   - 或者手动在菜单中选择

5. **开始控制**
   - 选择"手动控制"或"自动寻路"
   - 按照提示操作

## 🔧 高级配置

### 修改波特率（如果需要）

**下位机 (Core/Src/usart.c)**：
```c
huart3.Init.BaudRate = 115200;  // 修改为您需要的波特率
```

**上位机 (car_controller.cpp)**：
```cpp
SerialPort(const string& portName, int baudRate = 115200) {
    // 修改默认波特率
}
```

**注意**：下位机和上位机的波特率必须完全一致！

### 使用其他串口工具

如果不想使用提供的上位机程序，可以使用任何支持串口的工具：

- **PuTTY** (Windows/Linux)
- **串口助手** (Windows)
- **minicom** (Linux)
- **Arduino串口监视器**

只需配置相同的参数（115200, 8N1），然后发送命令即可。

## 📚 相关文档

- [上位机使用说明_CPP.md](./上位机使用说明_CPP.md) - C++上位机详细说明
- [硬件连接指南.md](./硬件连接指南.md) - 完整硬件连接说明
- [快速开始指南.md](./快速开始指南.md) - 快速入门指南

## ❓ 常见问题

**Q: USB转串口模块需要外部供电吗？**
A: 不需要。模块通过USB供电，但不要将VCC连接到STM32。

**Q: 可以同时使用PS2手柄和串口控制吗？**
A: 可以。通过PS2手柄的MODE键或串口命令 `CMD:MODE,0/1` 切换模式。

**Q: 串口通信距离有限制吗？**
A: USB转串口模块的通信距离受USB线长度限制（通常1-3米）。如需长距离，考虑使用RS485等方案。

**Q: 为什么我的串口号经常变化？**
A: Windows会根据插入顺序分配COM号。可以：
- 在设备管理器中手动设置固定COM号
- 或者每次运行程序时输入正确的COM号


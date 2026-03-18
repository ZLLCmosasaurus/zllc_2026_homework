# JLink RTT 功能介绍和使用

## `RTT`的介绍

`RTT`是`SEGGER`公司开发的一种技术，它允许在嵌入式应用程序中进行系统监控和交互式用户输入/输出。

相对于其他方案，`RTT`的优势很大：
 - 无需额外的接口，仅仅使用最基础的`SWDIO`和`SWCLK`便能使用。
 - 速度十分快，几乎没有任何性能开销，因为其工作方式就是读写内存。
 - 双向通信，主机与MCU之间可以互相收发数据。
 - 多通道支持

`RTT`的原理就是`RTT`库在`RAM`中创建了一个名为`SEGGER RTT Control Block`的控制块结构。在发送和读取数据时，本质上是对这块区域进行写和读。

## `RTT`的使用

### `RTT`库的导入

首先我们需要到`JLink`驱动的安装目录下的`Samples`文件夹中，在我的电脑上，这一路径为`C:\Program Files\SEGGER\JLink\Samples`。

在这个文件夹内部，有一个名为`RTT`的文件夹。新版的`JLink`驱动中没有这个文件夹，可以卸载然后安装旧版驱动来使用。亦或者是到[官方GitHub仓库](https://github.com/SEGGERMicro/RTT)中下载使用。

`RTT`文件夹中有一个压缩包，解压，然后将其中的`Config`文件夹和`RTT`文件夹复制到自己的项目下，并添加到项目资源。此时便可以使用了。

### `RTT`库的用法

在需要使用到`RTT`的地方，导入`SEGGER_RTT.h`。在调用`RTT`的`API`时，其会自动检测`RTT`是否初始化并进行初始化，所以不用显性的调用`SEGGER_RTT_Init`函数进行初始化。

#### 发送

`RTT`一共有三种向终端发送数据的方式，分别为`SEGGER_RTT_Write`、`SEGGER_RTT_WriteString`、`SEGGER_RTT_printf`。

说是发送其实并不准确，实际上是将数据写入到上行缓冲区。

如同函数名，这三者分别为将字节、字符串、以及格式化字符串写入到缓冲区。示例代码如下：
```c
uint8_t bytes[] = "Hello, World from SEGGER_RTT_Write!\n";
SEGGER_RTT_Write(0, bytes, strlen(bytes));
SEGGER_RTT_WriteString(0, "Hello, World from SEGGER_RTT_WriteString!\n");
SEGGER_RTT_printf(0, "Hello, World from SEGGER_RTT_printf on %d/%d/%d!\n", year, month, day);
```
将数据写入到缓冲区后，主机会读取缓冲区数据。

##### 添加`SEGGER_RTT_printf`对浮点数的输出支持

`RTT`库提供的`SEGGER_RTT_printf`函数默认并不支持浮点数，我们可以手动为其加上相关支持。

我们在库文件中找到`SEGGER_RTT_printf.c`文件。在其中的`SEGGER_RTT_vprintf`函数中的`switch-case`语句块的末尾，加上如下代码：
```c
case 'f':
case 'F':{
    float fv;
    fv = (float)va_arg(*pParamList, double);

    // 输出负号
    if (fv < 0) _StoreChar(&BufferDesc, '-');

    v = abs((int)fv);

    _PrintInt(&BufferDesc, v, 10u, NumDigits, FieldWidth, FormatFlags); // 输出整数部分
    _StoreChar(&BufferDesc, '.'); // 小数点

    // 输出小数部分（3位小数）
    // 若需要修改小数位数，修改下面`1000`和`3`这两个常数即可。
    // 例如修改为两位小数，只需要将`1000`改为`10^2`=`100`，将`3`改为`2`即可。
    v = abs((int)(fv * 1000));
    v = v % 1000;
    _PrintInt(&BufferDesc, v, 10u, 3, FieldWidth, FormatFlags);

    break;
}
```
不过，除了资源紧张以外，建议直接使用`printf`函数转发`RTT`输出。

##### `printf`转发`RTT`

在`JLink`驱动下的`RTT`文件夹内，还有一个名为`Syscalls`的文件夹。其中有四个文件，需要根据自身编译器类型选择合适的文件。

以`GCC`编译器为例，我们选择`SEGGER_RTT_Syscalls_GCC.c`文件复制到项目中合适路径下，并添加为项目资源。

当然，如果观察文件中内容就会发现，四个文件会根据编译环境自行决定是否被编译，也就是说，为了省事直接把整个文件夹直接加入到项目中也可以使用。

然后在文件中调用`printf`函数并编译，我们就能观察到效果了。注意`printf`的内容后面一定要加上`\n`，防止内容卡在缓冲区无法输出。

此时我们会发现，`printf`的大部分功能我们都能使用，但还是不能输出浮点数。这一点我们只需要在编译选项后面加上
```
-Wl,-u_printf_float
```
即可。

##### 带有样式的字符串输出

在`SEGGER_RTT.h`的文件末尾，可以看到许多基于`ANSI`的转义代码的宏。

要输出样式，我们可以直接用`ANSI`转义代码实现，也可以用预先设置好的宏。

```c
printf(RTT_CTRL_TEXT_RED RTT_CTRL_BG_YELLOW "Text color red, background blue\n");
```

#### 接收

接收和发送类似，也存在一个下行缓冲区。与接收相关的函数分别为`SEGGER_RTT_Read()`、`SEGGER_RTT_HasKey()`、`SEGGER_RTT_GetKey()`。

如同函数名，其作用分别为将下行缓冲区的数据读入、检测下行缓冲区是否有数据以及获取其中的一位数据。
可以从下面的例子中了解其用法。
```c
char acIn[4];
uint8_t NumBytes = sizeof(acIn);
NumBytes = SEGGER_RTT_Read(0, &acIn[0], NumBytes); // SEGGER_RTT_Read函数的返回值为缓冲区中的有效位数。
if (NumBytes) {
  AnalyzeInput(acIn);
}
```

```c
if (SEGGER_RTT_HasKey()) {
  int c = SEGGER_RTT_GetKey();
}
```

一个完整的接收示例如下：

首先定义缓冲数组。
```c
char RTT_BUFFER_DOWN[BUFFER_SIZE_DOWN];
```

然后声明并实现接收函数。

```c
void RTT_ProcessDown();

void RTT_ProcessDown()
{
    uint8_t NumBytes = BUFFER_SIZE_DOWN;
    NumBytes = SEGGER_RTT_Read(0, RTT_BUFFER_DOWN, NumBytes);
    if (NumBytes)
    {
        // 手动添加字符串终止符
        RTT_BUFFER_DOWN[NumBytes] = '\0';

        // 数据处理
        printf(RTT_CTRL_RESET "%d\n", atoi(RTT_BUFFER_DOWN));
    }
}
```

最后在主循环中调用。

```c
while (1)
{
    RTT_ProcessDown();
}
```

#### 终端切换

调用`SEGGER_RTT_SetTerminal`函数即可切换终端。

## 实例——基于`RTT`的日志系统

```c
#ifndef __LOG_H
#define __LOG_H

#include <stdio.h>
#include <SEGGER_RTT.h>

// 日志级别定义
#define LOG_LEVEL_DEBUG   0
#define LOG_LEVEL_INFO    1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_ERROR   3

// 当前日志级别（可以全局设置）
static int current_log_level = LOG_LEVEL_DEBUG;

// 获取颜色代码
#define LOG_COLOR(level) \
    (level == LOG_LEVEL_DEBUG) ? RTT_CTRL_TEXT_CYAN : \
    (level == LOG_LEVEL_INFO)  ? RTT_CTRL_TEXT_GREEN : \
    (level == LOG_LEVEL_WARN)  ? RTT_CTRL_TEXT_YELLOW : \
    (level == LOG_LEVEL_ERROR) ? RTT_CTRL_TEXT_RED : RTT_CTRL_RESET

// 获取级别名称
#define LOG_LEVEL_NAME(level) \
    (level == LOG_LEVEL_DEBUG) ? "DEBUG" : \
    (level == LOG_LEVEL_INFO)  ? "INFO" : \
    (level == LOG_LEVEL_WARN)  ? "WARN" : \
    (level == LOG_LEVEL_ERROR) ? "ERROR" : "UNKNOWN"

// 核心日志宏 - 使用do-while(0)包装以确保语法正确
#define LOG_PRINT(level, fmt, ...) do { \
    if (level >= current_log_level) { \
        printf("%s[%s] " fmt "%s\n", \
               LOG_COLOR(level), \
               LOG_LEVEL_NAME(level), \
               ##__VA_ARGS__, \
               RTT_CTRL_RESET); \
    } \
} while(0)

// 便捷宏
#define LOG_DEBUG(fmt, ...) LOG_PRINT(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  LOG_PRINT(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  LOG_PRINT(LOG_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) LOG_PRINT(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif
```
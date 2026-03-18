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
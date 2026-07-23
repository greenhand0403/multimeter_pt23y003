#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdio.h>
#include "PT32Y003x.h"
// 调试信息打印开关
#define ENABLE_LOG 0

#if ENABLE_LOG

#define LOG_UART UART0
#define LOG_BUFFER_SIZE 64

extern char log_buffer[LOG_BUFFER_SIZE];

void UART_Driver(void);
void UART_SendString(const char *str);

#define LOGF(...)                                                     \
    do                                                                \
    {                                                                 \
        snprintf(log_buffer, LOG_BUFFER_SIZE, __VA_ARGS__);           \
        UART_SendString(log_buffer);                                  \
    } while (0)

#define LOGS(s)                                                       \
    do                                                                \
    {                                                                 \
        UART_SendString(s);                                           \
    } while (0)

#else

#define LOGF(...) do {} while (0)
#define LOGS(s)   do {} while (0)

#endif

#endif
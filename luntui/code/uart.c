#include "uart.h"


#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART0_RX_P00_0

uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;
#include <stdio.h>
#include <stdarg.h>

/**
 * @brief  自定义串口格式化打印函数
 * @param  uart_n : 串口模块号 (UART_0, UART_1 等)
 * @param  format : 格式化字符串 (同 printf)
 * @param  ...    : 可变参数
 */
void uart_printf(uart_index_enum uart_n, const char *format, ...)
{
    char buffer[256]; // 缓冲区，大小根据需要调整（注意栈溢出风险）
    va_list args;

    va_start(args, format);
    // 使用 vsnprintf 将格式化内容打印到 buffer 中
    // 使用 vsnprintf 比 vsprintf 更安全，防止缓冲区溢出
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 调用你库里的字符串发送函数
    uart_write_string(uart_n, buffer);
}
void uart_task()
{
        fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
        if(fifo_data_count != 0)                                                // 读取到数据了
        {
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
            uart_printf(UART_INDEX, "\r\nUART get data:");                // 输出测试信息
            uart_printf(UART_INDEX, fifo_get_data, fifo_data_count);      // 将读取到的数据发送出去
        }
}






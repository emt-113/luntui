#ifndef __UART_C__
#define __UART_C__
#include "bsp_app.h"
void uart_task();
void uart_printf(uart_index_enum uart_n, const char *format, ...);
#endif 







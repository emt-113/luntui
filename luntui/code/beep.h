#ifndef __BEEP_C__
#define __BEEP_C__
#include "bsp_app.h"


void beep_init();
typedef struct {
    uint32_t start_time;    // 开始响的时间
    uint32_t duration;      // 需要响的时长
    int8_t is_running;      // 0 为停止，1 为正在鸣叫
} Buzzer_Ctrl_t;

// 声明外部可访问的变量和函数
extern Buzzer_Ctrl_t buzzer_status;

void buzzer_start_beep(uint32_t ms);
void buzzer_task(void);
#endif 







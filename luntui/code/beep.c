#include "beep.h"

#define BUZZER_PIN              (P19_4) 
// 蜂鸣器状态结构体


// 实例化并初始化变量
Buzzer_Ctrl_t buzzer_status = {0, 0, 0};
void beep_init()
{
  gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}
// 触发提示音函数
void buzzer_start_beep(uint32_t ms) {
    gpio_set_level(BUZZER_PIN, GPIO_HIGH);  // 开启
    buzzer_status.start_time = timer_get(TC_TIME2_CH0);
    buzzer_status.duration = ms;
    buzzer_status.is_running = 1;           // 标记为运行中
}

// 轮询处理函数，放在 while(1) 中
void buzzer_task(void) {
    if (buzzer_status.is_running == 1) {
        uint32_t current_time = timer_get(TC_TIME2_CH0);
        
        // 计算时间差，判断是否到达设定时长
        if (current_time - buzzer_status.start_time >= buzzer_status.duration) {
            gpio_set_level(BUZZER_PIN, GPIO_LOW); // 关闭
            buzzer_status.is_running = 0;         // 标记为停止
        }
    }
}







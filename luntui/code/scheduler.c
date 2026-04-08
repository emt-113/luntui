#include "scheduler.h"
#include "bsp_app.h"
#include "auto_menu.h"          // 引入菜单系统
extern uint32_t uwtick;
void tuoluoyi_printf()
{

//    printf("%d,%.5f,%.5f,%.5f\r\n",g_Debug_PWM,icm_output.gyro_y,icm_output.pitch,g_Debug_Target_Gyro);
  printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",icm_output.pitch,icm_output.roll,icm_output.yaw,icm_output.gyro_y,icm_output.gyro_x,icm_output.gyro_z);
      printf("Target: %.2f,%.2f,%.2f\r\n",
             g_Debug_Target_Speed,
             g_Debug_Measure_Speed,
             g_Debug_Speed_Output);
//  printf ("%d,%.2f,%.2f,%.2f\r\n",g_Debug_PWM,g_Debug_Target_Gyro,icm_output.pitch,icm_output.gyro_y);
}
void cao()
{
    tft180_show_float(0,140,icm_output.pitch,3,2);
}
uint8_t scheduler_task_num = 0;
typedef struct {
    void (*task)(void);
    uint32_t   last_time;
    uint32_t   run_time;
}schedulaer_task_t;

schedulaer_task_t scheduler_task[]= {
  {uart_task,0,100},
  {tuoluoyi_printf,0,10},
  {buzzer_task,0,20},
  {cao,0,80},
  {dongdao_8,0,1},
  {key_task,0,15},
  {Menu_Refresh_Task,0,MENU_REFRESH_PERIOD_MS}  // 菜单刷新任务，100ms 周期
};
void scheduler_init(void)
{
    scheduler_task_num = sizeof(scheduler_task)/sizeof(scheduler_task[0]);
}

void scheduler_run(void)
{
    uint32_t time;
    uint8_t i;
	time =  timer_get(TC_TIME2_CH0);
    for(i=0;i<scheduler_task_num;i++)
    {
			if(time-scheduler_task[i].last_time>=scheduler_task[i].run_time)
			{
				scheduler_task[i].last_time=time;
				scheduler_task[i].task();
			}
    }
}







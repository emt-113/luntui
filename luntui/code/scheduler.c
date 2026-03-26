#include "scheduler.h"
#include "bsp_app.h"
#include "auto_menu.h"          // 引入菜单系统
extern uint32_t uwtick;
void tuoluoyi_printf()
{
//  printf("%d,%d,%d,%d,%d,%d\r\n",
//         imu660rb_acc_x,
//         imu660rb_acc_y,
//         imu660rb_acc_z,
//         imu660rb_gyro_x,
//         imu660rb_gyro_y,
//         imu660rb_gyro_z);
    printf("%d,%.5f,%.5f,%.5f\r\n",g_Debug_PWM,icm_output.gyro_y,icm_output.pitch,g_Debug_Target_Gyro);
//      printf("Target: %.2f,%.2f,%.2f\r\n",
//             g_Debug_Target_Speed,
//             g_Debug_Measure_Speed,
//             g_Debug_Speed_Output);
//  printf ("%d,%.2f,%.2f,%.2f\r\n",g_Debug_PWM,g_Debug_Target_Gyro,icm_output.pitch,icm_output.gyro_y);
}
void cao()
{
//  printf("time:%d\t\n",uwtick);
//  printf("%f,%f,%f\r\n",icm_data.yaw,icm_data.roll,icm_data.pitch);
//     tft180_show_string(0,0,"gyro_y:");tft180_show_float(50,0,icm_output.gyro_y,3,3);
    //tft180_show_string(0,20,"pitch:");
      tft180_show_float(0,140,icm_output.pitch,3,2);
//    tft180_show_string(0,40,"roll:");tft180_show_float(50,40,icm_output.roll,3,3);
//    tft180_show_string(0,60,"l:");tft180_show_int(20,60,motor_value.receive_left_speed_data  ,4);tft180_show_float(60,60,mECHANICAL_ZERO,2,2);
//    tft180_show_string(0,80,"r:");tft180_show_int(20,80,motor_value.receive_right_speed_data  ,4);tft180_show_float(60,60,g_Debug_Target_Speed,2,2);
//    tft180_show_string(0,100,"Tar_Gyro:");tft180_show_float(60,100,g_Debug_Target_Gyro,3,1);
//    tft180_show_string(0,120,"PWM");tft180_show_int(30,120,g_Debug_PWM,4);tft180_show_int(80,120,g_Debug_PWM_temp,4);
////    tft180_show_float(0,140,g_Debug_Measure_Speed,3,2);tft180_show_float(60,140,g_Debug_Speed_Output,2,2);
//  tft180_show_int(0,80,g_System_State,5);
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
  {cao,0,100},
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







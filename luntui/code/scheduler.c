#include "scheduler.h"
#include "bsp_app.h"
extern uint32_t uwtick;
void tuoluoyi_printf()
{
  printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",icm_data.yaw,icm_data.roll,icm_data.pitch,icm_data.acc_x,icm_data.acc_y,icm_data.acc_z,icm_data.gyro_x,icm_data.gyro_y,icm_data.gyro_z);
//  printf("%d,%d,%d,%d,%d,%d\r\n",imu660rb_acc_x,imu660rb_acc_y,imu660rb_acc_z,imu660rb_gyro_x,imu660rb_gyro_y,imu660rb_gyro_z);
}
void cao()
{
  printf("time:%d\t\n",uwtick);
//  printf("%f,%f,%f\r\n",icm_data.yaw,icm_data.roll,icm_data.pitch);
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
  {cao,0,1000}
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







#include "moter.h"



uint8_t run_flag=1;


void moter_task()
{
  int16_t left_duty,right_duty,left_duty_temp,right_duty_temp;
  if (run_flag==1)
  {
    if (ABS(icm_output.pitch)>30)
    {
      run_flag=0;
    }
    small_driver_set_duty(left_duty,-right_duty);
  }
  else 
  {
    small_driver_set_duty(0,0);
  }
}






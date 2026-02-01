/*
 * Servo.c
 *
 *  Created on: 2025年3月1日
 *      Author: 26766
 */
#include "Servo.h"
#include "math.h"
int16_t chushixiajiang  = 0  ; //+表示小车高度升高，-表示下降
extern uint8_t run_flag;
serv_control servo_1;
serv_control servo_2;
serv_control servo_3;
serv_control servo_4;

int jump_flag = 0;
void Servo_Init(void)
{
    servo_1.servo_pin = Servo_1;
    servo_1.fre=300;
    servo_1.center= servo_1_center;
    servo_1.dir = servo_1_dir;

    servo_2.servo_pin = Servo_2;
    servo_2.fre=300;
    servo_2.center= servo_2_center;
    servo_2.dir = servo_2_dir;

    servo_3.servo_pin = Servo_3;
    servo_3.fre=300;
    servo_3.center= servo_3_center;
    servo_3.dir = servo_3_dir;

    servo_4.servo_pin = Servo_4;
    servo_4.fre=300;
    servo_4.center= servo_4_center;
    servo_4.dir = servo_4_dir;

    servo_1.center = servo_1.center - chushixiajiang * servo_1.dir;
    servo_2.center = servo_2.center - chushixiajiang * servo_2.dir;
    servo_3.center = servo_3.center - chushixiajiang * servo_3.dir;
    servo_4.center = servo_4.center - chushixiajiang * servo_4.dir;

    servo_1.now_location = servo_1.center;
    servo_2.now_location = servo_2.center;
    servo_3.now_location = servo_3.center;
    servo_4.now_location = servo_4.center;


    pwm_init(servo_1.servo_pin, servo_1.fre, servo_1.now_location);
    pwm_init(servo_2.servo_pin, servo_2.fre, servo_2.now_location);
    pwm_init(servo_3.servo_pin, servo_3.fre, servo_3.now_location);
    pwm_init(servo_4.servo_pin, servo_4.fre, servo_4.now_location);

}
int16 xianfu(int16 pwm, int16_t max, int16_t min)
{
    if (pwm > max) pwm = max;
    if (pwm < min) pwm = min;
    else
    {
        return pwm;
    }
    return pwm;
}
void set_servo_duty(serv_control *servo, int16 duty)
{
    servo->now_location = xianfu(duty, 10000, -10000);
    pwm_set_duty(servo->servo_pin, duty);
}
void Servo_Control(serv_control *servo, int16 move_num)
{
    servo->now_location = servo->now_location +(servo->dir == 1 ? move_num : -move_num);
    servo->now_location = xianfu(servo->now_location, 10000, -10000);
    pwm_set_duty(servo->servo_pin, servo->now_location);
}
extern float singal_bridge;
extern uint8_t dianbian_flag;
uint8 sha_duo = 0;
void dynamic_steer_control(void)
{
    int16 steer_output_duty = 0;
    int16 steer_location[4] = {0};
    int16 steer_target_offest[4] = {0};
    if(run_flag == 1)
    {
        if (jump_flag == 0)
        {

        }
    }
    else
    {

        Servo_Control(&servo_1, xianfu(servo_1.center - servo_1.now_location, 10, -10) * servo_1.dir);
        Servo_Control(&servo_2, xianfu(servo_2.center - servo_2.now_location, 10, -10) * servo_2.dir);
        Servo_Control(&servo_3, xianfu(servo_3.center - servo_3.now_location, 10, -10) * servo_3.dir);
        Servo_Control(&servo_4, xianfu(servo_4.center - servo_4.now_location, 10, -10) * servo_4.dir);

    }

    //printf("%d, %d, %d, %d\t\n",xianfu(steer_target_offest[0], 10, -10), xianfu(steer_target_offest[1], 10, -10),xianfu(steer_target_offest[2], 10, -10),xianfu(steer_target_offest[3], 10, -10));

}


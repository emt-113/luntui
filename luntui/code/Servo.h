/*
 * Servo.h
 *
 *  Created on: 2025��3��1��
 *      Author: 26766
 */

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "bsp_app.h"
///////////////////////////////////////////////////////////////////////
typedef struct
{
   pwm_channel_enum servo_pin;
   int16  fre;
   int16  dir;
   int16  center;
   int16  now_location;

}serv_control;
#define Servo_1 TCPWM_CH57_P17_4    //��ǰ�� - ���³���λ������
#define Servo_2 TCPWM_CH58_P17_3    //�Һ��� + ���³���λ������
#define Servo_3 TCPWM_CH31_P10_3    //��ǰ�� + ���³���λ������
#define Servo_4 TCPWM_CH30_P10_2    //����� - ���³���λ������

#define servo_1_center 4650    //max���10000
#define servo_1_dir -1   // ����λ������

#define servo_2_center 4350  //4900
#define servo_2_dir 1  // ����λ������

#define servo_3_center 4400//4754
#define servo_3_dir 1  // ����λ������

#define servo_4_center 4600
#define servo_4_dir -1  // ����λ������

extern serv_control servo_1;
extern serv_control servo_2;
extern serv_control servo_3;
extern serv_control servo_4;
extern int8_t MAX_PWM_STEP;
void Servo_Init(void);

void set_servo_duty(serv_control *servo, int16 duty);

void Servo_Control(serv_control *servo, int16 move_num);

void dynamic_steer_control(void);
void shi();
void VMC_Update_All_Servos(float gao, float angle);

/////////////////////////////////////////////////////////////////////


#endif /* CODE_SERVO_H_ */

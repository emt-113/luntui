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

} serv_control;

/* 舵机引脚定义 */
#define Servo_1 TCPWM_CH57_P17_4    // 右前腿 - 导致车体位置升高
#define Servo_2 TCPWM_CH58_P17_3    // 右后腿 + 导致车体位置升高
#define Servo_3 TCPWM_CH31_P10_3    // 左前腿 + 导致车体位置升高
#define Servo_4 TCPWM_CH30_P10_2    // 左后腿 - 导致车体位置升高

/* 舵机中值与方向定义 */
#define servo_1_center 4650         // max最大10000
#define servo_1_dir -1              // 车体位置升高方向

#define servo_2_center 4350         // 初始值4900
#define servo_2_dir 1               // 车体位置升高方向

#define servo_3_center 4400         // 初始值4754
#define servo_3_dir 1               // 车体位置升高方向

#define servo_4_center 4600
#define servo_4_dir -1              // 车体位置升高方向

/* 外部变量声明 */
extern serv_control servo_1;
extern serv_control servo_2;
extern serv_control servo_3;
extern serv_control servo_4;

/* 函数原型声明 */
void Servo_Init(void);

void set_servo_duty(serv_control *servo, int16 duty);

void Servo_Control(serv_control *servo, int16 move_num);

void dynamic_steer_control(void);extern int16_t MAX_PWM_STEP;
void shi();
void VMC_Update_All_Servos(float gao, float angle);

/**
 * @brief VMC 更新所有舵机 (支持左右腿独立高度)
 *
 * @param height_R 右侧腿目标高度 (单位: cm) - 用于 Servo_1, Servo_2
 * @param height_L 左侧腿目标高度 (单位: cm) - 用于 Servo_3, Servo_4
 * @param angle 俯仰角 (单位: 度)
 *
 * @note 舵机映射关系:
 *       - Servo_1, Servo_2: 右腿 (前后) → 使用 height_R
 *       - Servo_3, Servo_4: 左腿 (前后) → 使用 height_L
 *
 * @note 功能说明:
 *       1. 右侧腿 (Servo_1, Servo_2) 使用 height_R
 *       2. 左侧腿 (Servo_3, Servo_4) 使用 height_L
 *       3. 所有舵机使用相同的俯仰角
 *       4. 带防抖处理 (PWM 变化率限制)
 *
 * @note 使用示例:
 *       VMC_Update_All_Servos_Roll(4.4f, 5.2f, 10.0f);
 *       // 右侧腿 4.4cm, 左侧腿 5.2f, 俯仰角 10°
 */
void VMC_Update_All_Servos_Roll(float height_R, float height_L, float angle);

/////////////////////////////////////////////////////////////////////


#endif /* CODE_SERVO_H_ */

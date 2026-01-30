/*
 * filt.h
 *
 *  Created on: 2024Äê12ÔÂ2ÈÕ
 *      Author: Administrator
 */

#ifndef CODE_FILT_H_
#define CODE_FILT_H_
#include "bsp_app.h"

extern float angle, angle_dot;
extern uint8 GyroOffset_init;


typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float pitch;
    float roll;
    float yaw;
} icm_param_t;


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;



extern double Lamp_fuyang;
typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
    float AXdata;
    float AYdata;
    float AZdata;
} gyro_param_t;


extern icm_param_t icm_data;
extern gyro_param_t GyroOffset;
float fast_sqrt(float x);
void gyroOffset_init(void);
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ICM_getValues();
void ICM_getEulerianAngles(void);
void huandao_yaw_correct(void);
extern float param_Kp ;
extern float param_Ki ;
#endif /* CODE_FILT_H_ */

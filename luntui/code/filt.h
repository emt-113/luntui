/*
 * filt.h
 *
 *  Created on: 2024��12��2��
 *      Author: Administrator
 */

#ifndef CODE_FILT_H_
#define CODE_FILT_H_
#include "bsp_app.h"



// 新增：输出数据结构体（用于调整符号后输出）
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
    // 零偏值（调试用）
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
} icm_output_t;



void imu_read();




extern icm_output_t icm_output;  // 新增输出变量


extern float param_Kp ;
extern float param_Ki ;

// 陀螺仪零偏校准
typedef struct {
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
    uint8_t is_calibrated;
} gyro_offset_t;

extern gyro_offset_t gyro_offset;

void gyro_calibrate(void);
void gyro_offset_compensate(void);

#endif /* CODE_FILT_H_ */

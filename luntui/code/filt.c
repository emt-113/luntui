#include "filt.h"

icm_output_t icm_output; 
static float imu660rc_gyro_dps(int16 gyro_value)
{
    return imu660rc_gyro_transition(gyro_value);
}

static float imu660rc_roll_deg(void)
{
    return imu660rc_pitch;
}

static float imu660rc_pitch_deg(void)
{
    return imu660rc_roll;
}

static float imu660rc_yaw_deg(void)
{
    return imu660rc_yaw;
}

gyro_offset_t gyro_offset = {0};

void imu_read()
{
  icm_output.acc_x=imu660rc_acc_y;
  icm_output.acc_y=imu660rc_acc_x;
  icm_output.acc_z=imu660rc_acc_z;
  icm_output.gyro_x=imu660rc_gyro_transition(imu660rc_gyro_y);
  icm_output.gyro_y=imu660rc_gyro_transition(imu660rc_gyro_x);
  icm_output.gyro_z=imu660rc_gyro_transition(imu660rc_gyro_z);
  icm_output.pitch=imu660rc_roll;
  icm_output.roll=imu660rc_pitch;
  icm_output.yaw=imu660rc_yaw;

  // 应用零偏补偿
  if (gyro_offset.is_calibrated) {
    icm_output.gyro_x -= gyro_offset.gyro_x_offset;
    icm_output.gyro_y -= gyro_offset.gyro_y_offset;
    icm_output.gyro_z -= gyro_offset.gyro_z_offset;
  }

//  // 死区滤波 - 静止时小噪声直接置0
//  #define GYRO_DEADZONE 1.4f
//  if (fabs(icm_output.gyro_x) < GYRO_DEADZONE) icm_output.gyro_x = 0;
//  if (fabs(icm_output.gyro_y) < GYRO_DEADZONE) icm_output.gyro_y = 0;
//  if (fabs(icm_output.gyro_z) < GYRO_DEADZONE) icm_output.gyro_z = 0;
}

/**
 * @brief 陀螺仪零偏校准
 * @note  需要保持IMU静止，采集200次数据求平均值
 * @note  注意：由于安装方向，x和y轴数据已互换，校准时要同步处理
 */
void gyro_calibrate(void)
{
    #define CALIB_SAMPLES    1000
    #define CALIB_DELAY_MS   5      // 每次采样间隔5ms，总共约1秒
    #define DISCARD_SAMPLES  100     // 丢弃前50个不稳定样本

    float sum_x = 0, sum_y = 0, sum_z = 0;

    // 1. 丢弃上电初期不稳定数据
    for (int i = 0; i < DISCARD_SAMPLES; i++) {
        volatile float dummy;
        dummy = imu660rc_gyro_transition(imu660rc_gyro_y);
        dummy = imu660rc_gyro_transition(imu660rc_gyro_x);
        dummy = imu660rc_gyro_transition(imu660rc_gyro_z);
        (void)dummy;
        system_delay_ms(CALIB_DELAY_MS);
    }

    // 2. 正式采样，每次间隔等待新数据
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        sum_x += imu660rc_gyro_transition(imu660rc_gyro_y);  // 对应 icm_output.gyro_x
        sum_y += imu660rc_gyro_transition(imu660rc_gyro_x);  // 对应 icm_output.gyro_y
        sum_z += imu660rc_gyro_transition(imu660rc_gyro_z);  // 对应 icm_output.gyro_z
        system_delay_ms(CALIB_DELAY_MS);
    }

    gyro_offset.gyro_x_offset = sum_x / CALIB_SAMPLES;
    gyro_offset.gyro_y_offset = sum_y / CALIB_SAMPLES;
    gyro_offset.gyro_z_offset = sum_z / CALIB_SAMPLES;
    gyro_offset.is_calibrated = 1;

    // 同时保存到 icm_output 方便查看
    icm_output.gyro_x_offset = gyro_offset.gyro_x_offset;
    icm_output.gyro_y_offset = gyro_offset.gyro_y_offset;
    icm_output.gyro_z_offset = gyro_offset.gyro_z_offset;
}




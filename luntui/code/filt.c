#include "filt.h"





#define delta_T     0.001f
#define M_PI        3.1415926f
//float icm_ay,icm_sy;



float I_ex, I_ey, I_ez;


quater_param_t Q_info = {1, 0, 0};


icm_param_t icm_data;
gyro_param_t GyroOffset;

uint8 GyroOffset_init = 0;

float imu_gyro_z_val=0;
float param_Kp = 2;
float param_Ki = 0.001;



float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


void gyroOffset_init(void)
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    for (uint16 i = 0; i < 500; ++i)
    {
        imu660rb_get_acc();                                     // 获取 IMU660RB 加速度计数据
        imu660rb_get_gyro();

        GyroOffset.Xdata += imu660rb_gyro_x;
        GyroOffset.Ydata += imu660rb_gyro_y;
        GyroOffset.Zdata += imu660rb_gyro_z;
        system_delay_ms(5);
    }

    GyroOffset.Xdata /= 500;
    GyroOffset.Ydata /= 500;
    GyroOffset.Zdata /= 500;
    GyroOffset.AXdata /= 500;
    GyroOffset.AYdata /= 500;
    GyroOffset.AZdata /= 500;

    GyroOffset_init = 1;
}

#define alpha         0.7f


void ICM_getValues()
{
    // 1. 加速度处理：根据头文件 4098 对应 ±8G 时的 1g
    // 这里的 alpha 0.3f 建议调大到 0.8f 左右，否则平衡轮腿响应太慢
    icm_data.acc_x = -((float)imu660rb_acc_x / 4098.0f);
    icm_data.acc_y = -((float)imu660rb_acc_y / 4098.0f);
    icm_data.acc_z = -((float)imu660rb_acc_z / 4098.0f);

    // 2. 陀螺仪处理：量程 ±2000dps -> 灵敏度 14.3 LSB/(°/s)
    // 正确公式：(原始值 - 零偏) / 灵敏度 * (PI/180) 得到 rad/s
    float gyro_sensitivity = 14.3f; 
    
    icm_data.gyro_x = ((float)imu660rb_gyro_x - GyroOffset.Xdata) / gyro_sensitivity * (M_PI / 180.0f);
    icm_data.gyro_y = ((float)imu660rb_gyro_y - GyroOffset.Ydata) / gyro_sensitivity * (M_PI / 180.0f);
    icm_data.gyro_z = ((float)imu660rb_gyro_z - GyroOffset.Zdata) / gyro_sensitivity * (M_PI / 180.0f);
// ==========================================================
    // 【修改】轴间耦合补偿
    // 根据你的数据计算得出：k ≈ -0.09
    // ==========================================================
    float coupling_k = -0.09f; 
    
    // 核心公式：从 Z 轴中减去 Y 轴的分量
    // 因为 k 是负数，这里实际上变成了 Z + (|k| * Y)，正好抵消掉你的反向漂移
    icm_data.gyro_z = icm_data.gyro_z - (icm_data.gyro_y * coupling_k);


    // ==========================================================
    // 【死区处理】（必须放在补偿之后！）
    // 数据显示你还存在约 -0.07 的静态零偏，所以死区不能太小
    // ==========================================================
    if (icm_data.gyro_z > -0.18f && icm_data.gyro_z < 0.18f) {
        icm_data.gyro_z = 0.0f;
    }
    // 同理，如果 X 和 Y 也有微小漂移，也可以加
    if (icm_data.gyro_x > -0.02f && icm_data.gyro_x < 0.02f) icm_data.gyro_x = 0.0f;
    if (icm_data.gyro_y > -0.02f && icm_data.gyro_y < 0.02f) icm_data.gyro_y = 0.0f;
}



void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
//    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //vz = (q0*q0-0.5f+q3 * q3) * 2;


    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;


    I_ex += halfT * ex;   // integral error scaled by Ki
    I_ey += halfT * ey;
    I_ez += halfT * ez;

  // 修改建议：不要让 Z 轴进行积分修正，因为它没有观测源（磁力计）
  // 加速度计产生的 ez 主要是 Pitch/Roll 的误差，修正到 Yaw 上只会导致乱漂
  gx = gx + param_Kp * ex + param_Ki * I_ex;
  gy = gy + param_Kp * ey + param_Ki * I_ey;



    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT


    // normalise quaternion
    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}



void ICM_getEulerianAngles(void)
{
      imu660rb_get_gyro();
      imu660rb_get_acc();                                     // 获取 IMU660RA 加速度计数据


    ICM_getValues();
    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;


    icm_data.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI; // pitch
    icm_data.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
    icm_data.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;//转向角
}


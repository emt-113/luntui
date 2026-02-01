/**
 * *****************************************************************************
 * @file    control.c
 * @brief   平衡小车核心控制层实现
 * @author  浮浮酱 (幽浮喵)
 * @date    2025-01-31
 * @version 1.0
 *
 * *****************************************************************************
 */

#include "control.h"

/* ========================================================================
 * 外部变量引用
 * ======================================================================== */

extern icm_output_t icm_output;                    // IMU 数据源 (定义在 filt.c)
extern serv_control servo_1, servo_2, servo_3, servo_4;  // 舵机控制结构体 (定义在 Servo.c)

/* ========================================================================
 * 全局调试变量定义
 * ======================================================================== */

float g_Debug_Pitch        = 0.0f;    // 当前俯仰角 (度)
float g_Debug_Gyro         = 0.0f;    // 当前角速度 (°/s)
float g_Debug_Target_Gyro  = 0.0f;    // 目标角速度 (外环输出, 串级控制专用)
int16_t g_Debug_PWM        = 0;       // 最终 PWM 输出
uint8_t g_System_State     = SYSTEM_STATE_STOP;  // 系统状态 (0=STOP, 1=RUN)

/* ========================================================================
 * 私有变量
 * ======================================================================== */

static pid_struct_t balance_angle_pid;  // 外环 - 角度环 PID 控制器
static pid_struct_t balance_gyro_pid;   // 内环 - 角速度环 PID 控制器
static uint32_t recovery_counter = 0;   // 自动恢复计数器

/* ========================================================================
 * 函数实现
 * ======================================================================== */

/**
 * @brief 控制系统初始化
 */
void control_init(void)
{
    // ============================================================
    // 初始化外环 - 角度环 PID 控制器
    // ============================================================
    // 输入: 角度误差 (Target_Pitch - Current_Pitch)
    // 输出: 目标角速度 (Target_Gyro)
    // 参数: kp, ki, kd, integral_max, output_max, output_min
    // 注意: 初始值全部设为 0.0f，安全第一！
    pid_init(&balance_angle_pid,
             ANGLE_KP,
             ANGLE_KI,
             ANGLE_KD,
             0.0f,                     // 积分限幅 (角度环暂不使用积分)
             100.0f,                   // 输出限幅 (目标角速度范围 ±100°/s)
             -100.0f);

    // ============================================================
    // 初始化内环 - 角速度环 PID 控制器
    // ============================================================
    // 输入: 角速度误差 (Target_Gyro - Current_Gyro)
    // 输出: 电机 PWM
    // 参数: kp, ki, kd, integral_max, output_max, output_min
    // 注意: 初始值全部设为 0.0f，安全第一！
    pid_init(&balance_gyro_pid,
             GYRO_KP,
             GYRO_KI,
             GYRO_KD,
             0.0f,                     // 积分限幅 (角速度环暂不使用积分)
             (float)PWM_MAX,
             (float)PWM_MIN);

    // ============================================================
    // 初始化系统状态
    // ============================================================
    g_System_State = SYSTEM_STATE_STOP;
    recovery_counter = 0;

    // 停机状态: 电机 PWM = 0
    small_driver_set_duty(0, 0);
}

/**
 * @brief 核心控制任务 (1ms 周期调用)
 *
 * 控制流程 (串级 PID 双环架构):
 *     1. 异常保护检测
 *     2. 自动恢复判断
 *     3. 外环（角度环）计算 → 输出目标角速度
 *     4. 内环（角速度环）计算 → 输出电机 PWM
 *     5. 电机混合输出 (考虑极性)
 *     6. 舵机中位锁死
 */
void control_run_1ms(void)
{
    // ============================================================
    // 1. 读取 IMU 数据
    // ============================================================
    float current_pitch = icm_output.pitch;       // 当前俯仰角 (度)
    float current_gyro  = icm_output.gyro_y;      // 当前角速度 (°/s)

    // 更新调试变量
    g_Debug_Pitch = current_pitch;
    g_Debug_Gyro  = current_gyro;

    // ============================================================
    // 2. 异常保护检测
    // ============================================================
    // 计算偏离机械零点的角度

    // 判断是否超过保护阈值
    if (current_pitch > TAITOUMAX_TILT || current_pitch < -DITOUMAX_TILT)
    {
        // 触发停机保护
        g_System_State = SYSTEM_STATE_STOP;
        recovery_counter = 0;  // 清零恢复计数器
    }

    // ============================================================
    // 3. 自动恢复判断
    // ============================================================
    if (g_System_State == SYSTEM_STATE_STOP)
    {
        // 停机状态: 检测角度是否回到正常范围
        if (current_pitch >= -10 && current_pitch <= 10)
        {
            recovery_counter++;  // 增加计数器

            // 持续时间达到阈值, 自动恢复运行
            if (recovery_counter >= RECOVERY_TIME)
            {
                g_System_State = SYSTEM_STATE_RUN;
                recovery_counter = 0;
            }
        }
        else
        {
            // 角度仍异常, 清零计数器
            recovery_counter = 0;
        }
    }

    // ============================================================
    // 4. 根据系统状态执行控制
    // ============================================================
    if (g_System_State == SYSTEM_STATE_STOP)
    {
        // --------------------------------------------------------
        // 停机状态: 电机 PWM = 0
        // --------------------------------------------------------
        small_driver_set_duty(0, 0);
        g_Debug_PWM = 0;
        g_Debug_Target_Gyro = 0.0f;

        // 停机时舵机回中位
        set_servo_duty(&servo_1, servo_1.center);
        set_servo_duty(&servo_2, servo_2.center);
        set_servo_duty(&servo_3, servo_3.center);
        set_servo_duty(&servo_4, servo_4.center);

        return;  // 提前退出, 不执行后续控制
    }

    // ============================================================
    // 5. 运行状态: 串级 PID 计算 (双环控制)
    // ============================================================

    // ------------------------------------------------------------
    // 外环 - 角度环 (Angle Loop / Outer Loop)
    // ------------------------------------------------------------
    /*
     * 输入:  角度误差 (MECHANICAL_ZERO - current_pitch)
     * 输出:  目标角速度 (target_gyro)
     *
     * 控制逻辑:
     * - 当车后仰 (Pitch > -10.6) → Error < 0 → 需要负向角速度来修正
     * - 当车前倾 (Pitch < -10.6) → Error > 0 → 需要正向角速度来修正
     */
    
    if (uwtick % 5 == 0)
    {
      float target_gyro = pid_compute(&balance_angle_pid, MECHANICAL_ZERO, current_pitch);
      g_Debug_Target_Gyro = target_gyro;
    }
    // 更新调试变量 (外环输出)
    

    // ------------------------------------------------------------
    // 内环 - 角速度环 (Gyro Loop / Inner Loop)
    // ------------------------------------------------------------
    /*
     * 输入:  角速度误差 (g_Debug_Target_Gyro - current_gyro)
     * 输出:  电机 PWM
     *
     * 控制逻辑:
     * - 目标角速度由外环给出
     * - 内环快速响应,控制实际角速度跟随目标值
     */
    // 陀螺仪数值缩放 (适配量级)
    float gyro_scaled = current_gyro * GYRO_SCALE;

    // 内环 PID 计算
    float pwm_output = pid_compute(&balance_gyro_pid, g_Debug_Target_Gyro, gyro_scaled);

    // ============================================================
    // 6. 电机混合输出 (考虑极性)
    // ============================================================
    /*
     * 电机极性说明 (关键!):
     *
     * 左轮: 正值(+) = 后退, 负值(-) = 前进
     * 右轮: 负值(-) = 后退, 正值(+) = 前进
     *
     * 控制逻辑推导:
     * - 当内环输出 PWM > 0 (代表车后仰，需要后退)
     *   → 左轮应给正值, 右轮应给负值
     *
     * - 当内环输出 PWM < 0 (代表车前倾，需要前进)
     *   → 左轮应给负值, 右轮应给正值
     *
     * 混合输出:
     * - Left_PWM  =  (int16_t)PWM_Out
     * - Right_PWM = -(int16_t)PWM_Out
     */
    if (pwm_output > 0) 
    {
        pwm_output += DEAD_ZONE; // 正向转动，加上死区值
    }
    else if (pwm_output < 0) 
    {
        pwm_output -= DEAD_ZONE; // 反向转动，减去死区值（负得更多）
    }
    int16_t left_pwm  =  -(int16_t)pwm_output;
    int16_t right_pwm =   (int16_t)pwm_output;
    // 发送到电机驱动
    small_driver_set_duty(left_pwm, right_pwm);

    // 更新调试变量
    g_Debug_PWM = right_pwm;  // 记录左轮 PWM

    // ============================================================
    // 7. 舵机中位锁死 (保持腿部刚度)
    // ============================================================
    /*
     * 舵机控制策略:
     * - 运行状态时, 持续将 4 个舵机锁死在对应的中值
     * - 确保腿部不软, 保持机械刚度
     */
    if (uwtick % 10000 == 0)
    {
      set_servo_duty(&servo_1, servo_1.center);
      set_servo_duty(&servo_2, servo_2.center);
      set_servo_duty(&servo_3, servo_3.center);
      set_servo_duty(&servo_4, servo_4.center);
    }

}

/**
 * @brief 系统状态控制 (手动启停)
 */
void control_set_state(uint8_t state)
{
    g_System_State = state;

    if (state == SYSTEM_STATE_STOP)
    {
        // 停机: 清除双环 PID 状态
        pid_reset(&balance_angle_pid);
        pid_reset(&balance_gyro_pid);
        recovery_counter = 0;
    }
    else
    {
        // 启动: 重置双环 PID 状态
        pid_reset(&balance_angle_pid);
        pid_reset(&balance_gyro_pid);
    }
}

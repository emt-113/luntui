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
#include "bsp_app.h"  // 需要获取 uwtick 等全局变量定义

/* ========================================================================
 * 外部变量引用
 * ======================================================================== */

extern icm_output_t icm_output;                    // IMU 数据源 (定义在 filt.c)
extern serv_control servo_1, servo_2, servo_3, servo_4;  // 舵机控制结构体 (定义在 Servo.c)
extern small_device_value_struct motor_value;      // 电机速度反馈 (定义在 small_driver_uart_control.c)

/* ========================================================================
 * 全局调试变量定义
 * ======================================================================== */

float g_Debug_Pitch        = 0.0f;    // 当前俯仰角 (度)
float g_Debug_Gyro         = 0.0f;    // 当前角速度 (°/s)
float g_Debug_Target_Gyro  = 0.0f;    // 目标角速度 (外环输出, 串级控制专用)
int16_t g_Debug_PWM_temp        = 0;
int16_t g_Debug_PWM        = 0;       // 最终 PWM 输出
int8_t g_System_State     = SYSTEM_STATE_STOP;  // 系统状态 (0=STOP, 1=RUN)
float mECHANICAL_ZERO  = MECHANICAL_ZERO;
int8_t speed_flag   = 1;
// ============================================================
// 速度环调试变量
// ============================================================
float g_Debug_Target_Speed   = 0.0f;   // 速度环目标速度 (外部设置,暂定0)
float g_Debug_Measure_Speed  = 0.0f;   // 速度环测量速度 (滤波后)
float g_Debug_Speed_Output   = 0.0f;   // 速度环PID输出 (VMC角度)
float g_Wheel_Speed_Danger_Limit = WHEEL_SPEED_DANGER_LIMIT_DEFAULT; // 平均速度保护阈值
uint8_t DEAD_ZONE=0;

/* ========================================================================
 * 横滚角保护调试变量定义
 * ======================================================================== */

float g_Debug_Roll         = 0.0f;    // 当前横滚角 (度)
float g_Debug_Roll_Display = 0.0f;    // 当前横滚角显示值 (度, 已补偿)
float g_Debug_Height_L     = 5.9f;    // 左侧腿高度 (cm)
float g_Debug_Height_R     = 5.9f;    // 右侧腿高度 (cm)
int8_t g_Debug_Roll_State  = 0;       // 横滚角保护状态 (0=死区, 1=左侧代偿, 2=右侧代偿, 3=危险)

/* ========================================================================
 * 横滚角保护可调参数 (用于菜单调节)
 * ======================================================================== */

float g_Roll_Kp = ROLL_KP;             // 横滚角比例系数 (初始值从宏定义读取)
float g_Roll_Ki = ROLL_KI;             // 横滚角积分系数 (初始值从宏定义读取)
float g_Roll_Kd = ROLL_KD;             // 横滚角微分系数 (初始值从宏定义读取)

/* ========================================================================
 * 私有变量
 * ======================================================================== */

 pid_struct_t balance_angle_pid;  // 外环 - 角度环 PID 控制器
 pid_struct_t balance_gyro_pid;   // 内环 - 角速度环 PID 控制器
 pid_struct_t speed_pid;          // 速度环 PID 控制器
static uint32_t recovery_counter = 0;   // 自动恢复计数器
static float speed_filtered = 0.0f;     // 滤波后的速度 (一阶低通)

/* ========================================================================
 * 横滚角保护私有变量
 * ======================================================================== */

static float roll_filtered = 0.0f;        // 滤波后的横滚角
static float roll_last = 0.0f;            // 上一次的横滚角 (用于计算微分项)
static float roll_integral = 0.0f;        // 横滚角积分项
static uint8_t roll_control_active = 0;   // 横滚补偿是否处于激活状态(滞回控制)
static float height_L_filtered = 4.4f;    // 滤波后的左侧高度
static float height_R_filtered = 4.4f;    // 滤波后的右侧高度

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
             ANGLE_PID_INTEGRAL_MAX,
             ANGLE_PID_OUTPUT_MAX,
             ANGLE_PID_OUTPUT_MIN);
    pid_set_enable(&balance_angle_pid, ANGLE_PID_ENABLE_P, ANGLE_PID_ENABLE_I, ANGLE_PID_ENABLE_D);
    pid_set_integral_limit(&balance_angle_pid, ANGLE_PID_INTEGRAL_MIN, ANGLE_PID_INTEGRAL_MAX);
    pid_set_integral_step_limit(&balance_angle_pid, ANGLE_PID_I_STEP_MIN, ANGLE_PID_I_STEP_MAX);
    pid_set_anti_windup(&balance_angle_pid, ANGLE_PID_ANTI_WINDUP_ENABLE);
    pid_set_derivative_mode(&balance_angle_pid, ANGLE_PID_D_MODE);
    pid_set_derivative_lpf(&balance_angle_pid, ANGLE_PID_D_LPF_ALPHA);
    pid_set_output_rate_limit(&balance_angle_pid,
                              ANGLE_PID_RATE_LIMIT_ENABLE,
                              ANGLE_PID_RATE_DOWN,
                              ANGLE_PID_RATE_UP);

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
             GYRO_PID_INTEGRAL_MAX,
             GYRO_PID_OUTPUT_MAX,
             GYRO_PID_OUTPUT_MIN);
    pid_set_enable(&balance_gyro_pid, GYRO_PID_ENABLE_P, GYRO_PID_ENABLE_I, GYRO_PID_ENABLE_D);
    pid_set_integral_limit(&balance_gyro_pid, GYRO_PID_INTEGRAL_MIN, GYRO_PID_INTEGRAL_MAX);
    pid_set_integral_step_limit(&balance_gyro_pid, GYRO_PID_I_STEP_MIN, GYRO_PID_I_STEP_MAX);
    pid_set_anti_windup(&balance_gyro_pid, GYRO_PID_ANTI_WINDUP_ENABLE);
    pid_set_derivative_mode(&balance_gyro_pid, GYRO_PID_D_MODE);
    pid_set_derivative_lpf(&balance_gyro_pid, GYRO_PID_D_LPF_ALPHA);
    pid_set_output_rate_limit(&balance_gyro_pid,
                              GYRO_PID_RATE_LIMIT_ENABLE,
                              GYRO_PID_RATE_DOWN,
                              GYRO_PID_RATE_UP);

    // ============================================================
    // 初始化速度环 PID 控制器
    // ============================================================
    // 输入: 速度误差 (Target_Speed - Measure_Speed)
    // 输出: VMC 目标角度
    // 参数: kp, ki, kd, integral_max, output_max, output_min
    // 注意: 初始值全部设为 0.0f，安全第一！
    pid_init(&speed_pid,
             SPEED_KP,
             SPEED_KI,
             SPEED_KD,
             SPEED_PID_INTEGRAL_MAX,
             SPEED_PID_OUTPUT_MAX,
             SPEED_PID_OUTPUT_MIN);
    pid_set_enable(&speed_pid, SPEED_PID_ENABLE_P, SPEED_PID_ENABLE_I, SPEED_PID_ENABLE_D);
    pid_set_integral_limit(&speed_pid, SPEED_PID_INTEGRAL_MIN, SPEED_PID_INTEGRAL_MAX);
    pid_set_integral_step_limit(&speed_pid, SPEED_PID_I_STEP_MIN, SPEED_PID_I_STEP_MAX);
    pid_set_anti_windup(&speed_pid, SPEED_PID_ANTI_WINDUP_ENABLE);
    pid_set_derivative_mode(&speed_pid, SPEED_PID_D_MODE);
    pid_set_derivative_lpf(&speed_pid, SPEED_PID_D_LPF_ALPHA);
    pid_set_output_rate_limit(&speed_pid,
                              SPEED_PID_RATE_LIMIT_ENABLE,
                              SPEED_PID_RATE_DOWN,
                              SPEED_PID_RATE_UP);

    // ============================================================
    // 初始化系统状态
    // ============================================================
    g_System_State = SYSTEM_STATE_STOP;
    recovery_counter = 0;

    // 停机状态: 电机 PWM = 0
    small_driver_set_duty(0, 0);
}
uint8_t start_flag=0;
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
    float current_roll  = icm_output.roll + ROLL_DISPLAY_OFFSET;  // 当前横滚角 (度, 已补偿)
    float current_speed_avg = (motor_value.receive_left_speed_data -
                               motor_value.receive_right_speed_data) / 2.0f;

    // 更新调试变量
    g_Debug_Pitch = current_pitch;
    g_Debug_Gyro  = current_gyro;
    g_Debug_Roll_Display = current_roll;

    // ============================================================
    // 2. 横滚角危险区检测 (新增)
    // ============================================================
    if (fabs(current_roll) > ROLL_DANGER_ZONE)
    {
        // 触发停机保护
        g_System_State = SYSTEM_STATE_STOP;
    }

    // ============================================================
    // 3. 俯仰角异常保护检测 (现有)
    // ============================================================
    if (current_pitch > TAITOUMAX_TILT || current_pitch < -DITOUMAX_TILT)
    {
        g_System_State = SYSTEM_STATE_STOP;
    }

    // ============================================================
    // 4. 平均速度异常保护检测
    // ============================================================
    if (fabs(current_speed_avg) > g_Wheel_Speed_Danger_Limit)
    {
        g_System_State = SYSTEM_STATE_STOP;
    }

    // ============================================================
    // 5. 根据系统状态执行控制
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
        VMC_Update_All_Servos_Roll(ROLL_BASE_HEIGHT,ROLL_BASE_HEIGHT,0);

        return;  // 提前退出, 不执行后续控制
    }

    // ============================================================
    // 6. 运行状态: 串级 PID 计算 (双环控制)
    // ============================================================

    // ------------------------------------------------------------
    // 外环 - 角度环 (Angle Loop / Outer Loop)
    // ------------------------------------------------------------
    /*
     * 输入:  角度误差 (mECHANICAL_ZERO - current_pitch)
     * 输出:  目标角速度 (target_gyro)
     *
     * 控制逻辑:
     * - 当车后仰 (Pitch > -10.6) → Error < 0 → 需要负向角速度来修正
     * - 当车前倾 (Pitch < -10.6) → Error > 0 → 需要正向角速度来修正
     */
    if (uwtick % 20 == 0 && speed_flag == 1)
    {
      speed_control_run();
    }
    if (uwtick % 5 == 0)
    {
      float target_gyro = pid_compute(&balance_angle_pid, mECHANICAL_ZERO, current_pitch, ANGLE_LOOP_DT_S);
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
    // 内环 PID 计算（单位统一为 deg/s，不再做经验缩放）
    float pwm_output = pid_compute(&balance_gyro_pid, g_Debug_Target_Gyro, current_gyro, GYRO_LOOP_DT_S);

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
    g_Debug_PWM_temp=pwm_output;
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

}
 float pid_output;
/**
 * @brief 速度环控制任务 (20ms周期调用)
 *
 * 控制流程:
 *     1. 读取左右轮速度并计算平均值
 *     2. 一阶低通滤波 (系数 0.3)
 *     3. PID计算 → VMC目标角度
 *     4. 根据系统状态执行VMC或重置
 */
void speed_control_run(void)
{
    // ============================================================
    // 1. 计算平均速度
    // ============================================================
    float speed_avg = (motor_value.receive_left_speed_data -
                       motor_value.receive_right_speed_data) / 2.0f;

    // ============================================================
    // 2. 一阶低通滤波
    // ============================================================
    // 滤波公式: y[n] = α * x[n] + (1-α) * y[n-1]
    // α = 0.3 (新数据权重)
    speed_filtered = 0.3f * speed_avg + 0.7f * speed_filtered;

    // 更新调试变量
    g_Debug_Measure_Speed = speed_filtered;
  
    // ============================================================
    // 3. 根据系统状态执行控制
    // ============================================================
    if (g_System_State == SYSTEM_STATE_RUN)
    {
        // --------------------------------------------------------
        // 运行状态: 计算速度环PID → VMC
        // --------------------------------------------------------

        // 速度环PID计算 (目标速度暂定0,后续可通过遥控/按键修改)
         pid_output = -pid_compute(&speed_pid, g_Debug_Target_Speed, speed_filtered, SPEED_LOOP_DT_S);

        // 输出限幅 (PID库已自动限幅,这里再次确保安全)
        if (pid_output > SPEED_OUT_MAX) pid_output = SPEED_OUT_MAX;
        if (pid_output < SPEED_OUT_MIN) pid_output = SPEED_OUT_MIN;

        // 更新调试变量
        g_Debug_Speed_Output = pid_output;

        // ========================================================
        // [修改] 横滚角防护计算 (新增)
        // ========================================================
        float height_L, height_R;
        Roll_Protection_Compute(icm_output.roll, &height_L, &height_R);

        // 执行VMC控制 (注意参数顺序: height_R 在前, height_L 在后)
        VMC_Update_All_Servos_Roll(height_R, height_L, pid_output);
    }
    else
    {
        // --------------------------------------------------------
        // 停机状态: 重置PID积分,舵机归中
        // --------------------------------------------------------

        // 重置速度环PID积分项 (防止累积误差)
        pid_reset(&speed_pid);

        // 舵机回中位 (保持机械零点)
        set_servo_duty(&servo_1, servo_1.center);
        set_servo_duty(&servo_2, servo_2.center);
        set_servo_duty(&servo_3, servo_3.center);
        set_servo_duty(&servo_4, servo_4.center);

        // 清零调试变量
        g_Debug_Speed_Output = 0.0f;
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

/**
 * @brief 横滚角防护计算函数 (20ms 周期调用)
 *
 * @note 控制策略 (单边不对称代偿):
 *       1. 死区保护: |Roll| ≤ 3° → 左右腿均保持 4.4cm
 *       2. 动态代偿: 3° < |Roll| ≤ 45° → 单边伸长补偿
 *          - Roll < 0° (左侧偏低) → 左腿伸长，右腿保持 4.4cm
 *          - Roll > 0° (右侧偏高) → 右腿伸长，左腿保持 4.4cm
 *       3. 危险区: |Roll| > 45° → 触发停机保护
 *
 * @note 实现细节:
 *       - PID 控制计算伸长量
 *       - 一阶低通滤波平滑输出
 *       - 多重限幅保护
 */
void Roll_Protection_Compute(float roll_current, float* height_L, float* height_R)
{
    float roll_corrected = roll_current + ROLL_DISPLAY_OFFSET;
    float abs_roll = 0.0f;
    float zone_scale = 1.0f;

    // ============================================================
    // 1. 一阶低通滤波 (平滑 Roll 角输入)
    // ============================================================
    // 滤波公式: y[n] = α * x[n] + (1-α) * y[n-1]
    roll_filtered = ROLL_FILTER_ALPHA * roll_corrected + (1.0f - ROLL_FILTER_ALPHA) * roll_filtered;

    // 更新调试变量
    g_Debug_Roll = roll_filtered;
    abs_roll = fabs(roll_filtered);

    // ============================================================
    // 2. 两段控制 (分界线: 1.5°)
    //    |roll| >= 1.5°: 全量 PID
    //    |roll| <  1.5°: 继续计算,按比例衰减并衰减积分
    // ============================================================
    if (abs_roll < ROLL_DEAD_ZONE)
    {
        if (ROLL_DEAD_ZONE > 0.0f)
        {
            zone_scale = abs_roll / ROLL_DEAD_ZONE;
        }
        roll_integral *= ROLL_INT_DECAY_FACTOR;
        if (zone_scale < 0.0f) zone_scale = 0.0f;
        if (zone_scale > 1.0f) zone_scale = 1.0f;
    }

    // ============================================================
    // 3. 危险区检测 (±45°)
    // ============================================================
    if (fabs(roll_filtered) > ROLL_DANGER_ZONE)
    {
        // 触发停机保护 (在 control_run_1ms 中处理)
        g_Debug_Roll_State = 3;  // 危险状态

        // 停机时保持基础高度
        *height_L = ROLL_BASE_HEIGHT;
        *height_R = ROLL_BASE_HEIGHT;
        roll_integral = 0.0f;
        roll_control_active = 0U;

        return;  // 提前退出
    }

    // ============================================================
    // 4. 动态代偿区 (PID 控制)
    // ============================================================

    // 计算微分项 (D 项)
    float roll_derivative = roll_filtered - roll_last;
    roll_last = roll_filtered;

    // 根据横滚角符号决定哪一侧需要补偿
    if (roll_filtered < 0.0f)
    {
        // --------------------------------------------------------
        // 左侧偏低 (Roll < 0°)
        // → 左腿需要伸长，右腿保持基础高度
        // --------------------------------------------------------

        g_Debug_Roll_State = 1;  // 左侧代偿状态

        // PID 控制计算 (左侧: 误差为正)
        float error = (-roll_filtered) * zone_scale;  // 转换为正值并做区间衰减
        float error_derivative = (-roll_derivative) * zone_scale;
        float integral_step = error * ROLL_CONTROL_DT;
        float integral_next = 0.0f;

        if (integral_step > ROLL_INT_STEP_MAX) integral_step = ROLL_INT_STEP_MAX;
        if (integral_step < -ROLL_INT_STEP_MAX) integral_step = -ROLL_INT_STEP_MAX;
        integral_next = roll_integral + integral_step;
        if (integral_next > ROLL_INT_MAX) integral_next = ROLL_INT_MAX;
        if (integral_next < 0.0f) integral_next = 0.0f;
        float pid_output = g_Roll_Kp * error + g_Roll_Ki * integral_next + g_Roll_Kd * error_derivative;

        // 输出限幅 + 积分限幅 (防风up)
        if (pid_output > ROLL_OUT_MAX)
        {
            pid_output = ROLL_OUT_MAX;
        }
        else if (pid_output < ROLL_OUT_MIN)
        {
            pid_output = ROLL_OUT_MIN;
        }
        roll_integral = integral_next;

        // 左侧腿伸长 = 基础高度 + PID 输出
        float height_L_target = ROLL_BASE_HEIGHT + pid_output;

        // 机械限幅
        if (height_L_target > ROLL_MAX_HEIGHT) height_L_target = ROLL_MAX_HEIGHT;

        // 平滑滤波 (防止高度突变)
        height_L_filtered = ROLL_FILTER_ALPHA * height_L_target + (1.0f - ROLL_FILTER_ALPHA) * height_L_filtered;
        height_R_filtered = ROLL_BASE_HEIGHT;  // 右侧保持基础高度

        *height_L = height_L_filtered;
        *height_R = height_R_filtered;
    }
    else
    {
        // --------------------------------------------------------
        // 右侧偏低 (Roll > 0°)
        // → 右腿需要伸长，左腿保持基础高度
        // --------------------------------------------------------

        g_Debug_Roll_State = 2;  // 右侧代偿状态

        // PID 控制计算 (右侧: 误差为正)
        float error = roll_filtered * zone_scale;  // 已经是正值
        float error_derivative = roll_derivative * zone_scale;
        float integral_step = error * ROLL_CONTROL_DT;
        float integral_next = 0.0f;

        if (integral_step > ROLL_INT_STEP_MAX) integral_step = ROLL_INT_STEP_MAX;
        if (integral_step < -ROLL_INT_STEP_MAX) integral_step = -ROLL_INT_STEP_MAX;
        integral_next = roll_integral + integral_step;
        if (integral_next > ROLL_INT_MAX) integral_next = ROLL_INT_MAX;
        if (integral_next < 0.0f) integral_next = 0.0f;
        float pid_output = g_Roll_Kp * error + g_Roll_Ki * integral_next + g_Roll_Kd * error_derivative;

        // 输出限幅 + 积分限幅 (防风up)
        if (pid_output > ROLL_OUT_MAX)
        {
            pid_output = ROLL_OUT_MAX;
        }
        else if (pid_output < ROLL_OUT_MIN)
        {
            pid_output = ROLL_OUT_MIN;
        }
        roll_integral = integral_next;

        // 右侧腿伸长 = 基础高度 + PID 输出
        float height_R_target = ROLL_BASE_HEIGHT + pid_output;

        // 机械限幅
        if (height_R_target > ROLL_MAX_HEIGHT) height_R_target = ROLL_MAX_HEIGHT;

        // 平滑滤波 (防止高度突变)
        height_R_filtered = ROLL_FILTER_ALPHA * height_R_target + (1.0f - ROLL_FILTER_ALPHA) * height_R_filtered;
        height_L_filtered = ROLL_BASE_HEIGHT;  // 左侧保持基础高度

        *height_L = height_L_filtered;
        *height_R = height_R_filtered;
    }

    // 低角度且高度已接近基础值时，才真正退出补偿
    if (abs_roll < ROLL_DEAD_ZONE &&
        fabs(height_L_filtered - ROLL_BASE_HEIGHT) < ROLL_RELEASE_HEIGHT_EPS &&
        fabs(height_R_filtered - ROLL_BASE_HEIGHT) < ROLL_RELEASE_HEIGHT_EPS)
    {
        roll_control_active = 0U;
        roll_integral = 0.0f;
        g_Debug_Roll_State = 0;
    }
}

/**
 * *****************************************************************************
 * @file    control.h
 * @brief   平衡小车核心控制层头文件
 * @author  浮浮酱 (幽浮喵)
 * @date    2025-01-31
 * @version 1.0
 *
 * ==============================================================================
 * 模块说明:
 *     平衡小车的核心控制逻辑，包括直立环、异常保护、自动恢复等功能
 *
 * 核心功能:
 *     1. 直立环控制 (PD 控制)
 *     2. 异常保护与自动恢复
 *     3. 舵机中位锁死 (保持腿部刚度)
 *     4. 调试变量输出
 *
 * 控制流程:
 *     1. 异常检测 → 2. 自动恢复判断 → 3. 直立环计算 → 4. 电机混合输出
 * ==============================================================================
 *
 * *****************************************************************************
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "pid.h"      // 必须先包含 pid.h（pid_struct_t 定义）
#include "zf_common_headfile.h"  // 底层头文件

/* ========================================================================
 * 硬件参数宏定义
 * ======================================================================== */

/**
 * @brief 机械零点 (物理平衡点)
 *
 * @note 这是小车的自然平衡位置，需要通过实验测定
 *       当前值: -10.6° (负值表示车头自然下垂)
 */
#define MECHANICAL_ZERO            (-7.75f)

/**
 * @brief 最大倾斜角度 (异常保护阈值)
 *
 * @note 当 |pitch - MECHANICAL_ZERO| > MAX_TILT 时触发停机保护
 *       单位: 度 (°)
 */
#define TAITOUMAX_TILT                   (30.0f) //抬头
#define DITOUMAX_TILT                   (32.0f)  //低头
/**
 * @brief 自动恢复角度阈值
 *
 * @note 当角度回到 [MECHANICAL_ZERO - RECOVERY_THRESHOLD, MECHANICAL_ZERO + RECOVERY_THRESHOLD]
 *       范围内，并持续 RECOVERY_TIME ms 后自动恢复运行
 *       单位: 度 (°)
 */
#define RECOVERY_THRESHOLD         (5.0f)

/**
 * @brief 自动恢复时间阈值
 *
 * @note 角度需要在正常范围内持续的时间 (单位: 毫秒)
 */
#define RECOVERY_TIME              (2000)

/**
 * @brief 平均速度危险阈值
 *
 * @note 当速度环使用的平均速度绝对值超过该阈值时，触发停机保护
 *       计算方式: (left_speed - right_speed) / 2
 */
#define WHEEL_SPEED_DANGER_LIMIT_DEFAULT   (1700.0f)

/**
 * @brief 陀螺仪放大系数
 *
 * @note gyro_y 数值范围较小 (约 -5~5)，用于调整量级
 *       建议初始值: 1.0f，根据实际调试效果调整
 */
#define GYRO_SCALE                 (1.0f)  // 已停用：控制环统一使用 deg/s，不做经验缩放
 
 /**
 * @brief 定义死区补偿值
 *
 * @note DEAD_ZONE 
 */
extern uint8_t DEAD_ZONE ;// 定义死区补偿值 (根据你的实测230，建议设为210左右，避免原地高频震荡)
extern float g_Wheel_Speed_Danger_Limit; // 平均速度保护阈值，供菜单调节

/* ========================================================================
 * PID 参数宏定义 (串级双环控制)
 * ======================================================================== */

#define GYRO_LOOP_DT_S            (0.001f)
#define ANGLE_LOOP_DT_S           (0.005f)
#define SPEED_LOOP_DT_S           (0.020f)
#define ROLL_LOOP_DT_S            (0.020f)

/**
 * @brief PWM 输出限幅
 *
 * @note 保护电机驱动，防止过流损坏
 *       small_driver_set_duty() 范围: -10000 ~ 10000
 */
#define PWM_MAX                   (4000)
#define PWM_MIN                   (-4000)

// ============================================================
// 外环 - 俯仰角环 (pitch -> target_gyro)
// ============================================================
#define ANGLE_PID_ENABLE_P                PID_TRUE
#define ANGLE_PID_ENABLE_I                PID_TRUE
#define ANGLE_PID_ENABLE_D                PID_TRUE

/**
 * @brief 角度环比例系数
 *
 * @note 控制角度回归平衡位置的能力
 *       初始值设为 0.0f，需要从 0 开始逐步增加调试
 *       典型范围: 0.1 ~ 10.0
 *
 *       调试建议:
 *       - 先调节此参数,使小车能够勉强维持直立
 *       - 过大会导致振荡,过小响应太慢
 */
#define ANGLE_KP                          (37.9f)

/**
 * @brief 角度环微分系数
 *
 * @note 预测角度变化趋势,提前抑制偏差
 *       初始值设为 0.0f，根据调试效果调整
 *       典型范围: 0.0 ~ 1.0
 *
 *       注意: 角度环一般不需要太大的 D 项
 */
#define ANGLE_KD                          (0.11)

/**
 * @brief 角度环积分系数
 *
 * @note 消除角度稳态误差
 *       初始值设为 0.0f，一般不需要积分
 *       保留接口供特殊场景使用
 */
#define ANGLE_KI                          (2.5f)
#define ANGLE_PID_OUTPUT_MAX              (150.0f)
#define ANGLE_PID_OUTPUT_MIN              (-150.0f)
#define ANGLE_PID_INTEGRAL_MAX   (80.0f)
#define ANGLE_PID_INTEGRAL_MIN   (-80.0f)
#define ANGLE_PID_I_STEP_MAX     (6)
#define ANGLE_PID_I_STEP_MIN     (-6)
#define ANGLE_PID_ANTI_WINDUP_ENABLE      PID_TRUE
#define ANGLE_PID_D_MODE                  PID_DERIVATIVE_ON_MEASUREMENT
#define ANGLE_PID_D_LPF_ALPHA             (0.6)
#define ANGLE_PID_RATE_LIMIT_ENABLE       PID_FALSE
#define ANGLE_PID_RATE_UP                 (0.0f)
#define ANGLE_PID_RATE_DOWN               (0.0f)

// ============================================================
// 内环 - 角速度环 (gyro_y -> pwm_output)
// 初调建议只开 P
// ============================================================

#define GYRO_PID_ENABLE_P                 PID_TRUE
#define GYRO_PID_ENABLE_I                 PID_TRUE
#define GYRO_PID_ENABLE_D                 PID_TRUE

/**
 * @brief 角速度环比例系数
 *
 * @note 控制角速度快速跟随目标值
 *       初始值设为 0.0f，需要从 0 开始逐步增加调试
 *       典型范围: 1.0 ~ 100.0
 *
 *       调试建议:
 *       - 在角度环调好后,再调节此参数
 *       - 此参数影响系统的阻尼特性
 *       - 过大会导致高频振荡,过小响应慢
 */
#define GYRO_KP                           (15.5f)

/**
 * @brief 角速度环微分系数
 *
 * @note 抑制角速度的高频振荡
 *       初始值设为 0.0f，一般不需要
 *       保留接口供特殊场景使用
 */
#define GYRO_KD                           (0.0f)

/**
 * @brief 角速度环积分系数
 *
 * @note 消除角速度稳态误差
 *       初始值设为 0.0f，根据调试效果调整
 *       典型范围: 0.0 ~ 10.0
 *
 *       注意: 积分项可能导致超调和振荡,谨慎使用
 */
#define GYRO_KI                           (0.0f)
#define GYRO_PID_OUTPUT_MAX               ((float)PWM_MAX)
#define GYRO_PID_OUTPUT_MIN               ((float)PWM_MIN)
#define GYRO_PID_INTEGRAL_MAX             (0.0f)
#define GYRO_PID_INTEGRAL_MIN             (0.0f)
#define GYRO_PID_I_STEP_MAX               (0.0f)
#define GYRO_PID_I_STEP_MIN               (0.0f)
#define GYRO_PID_ANTI_WINDUP_ENABLE       PID_TRUE
#define GYRO_PID_D_MODE                   PID_DERIVATIVE_ON_MEASUREMENT
#define GYRO_PID_D_LPF_ALPHA              (0.4)
#define GYRO_PID_RATE_LIMIT_ENABLE        PID_FALSE
#define GYRO_PID_RATE_UP                  (0.0f)
#define GYRO_PID_RATE_DOWN                (0.0f)

// ============================================================
// 速度环 (speed -> VMC angle)
// 慢环建议先只开 P
// ============================================================

#define SPEED_PID_ENABLE_P                PID_TRUE
#define SPEED_PID_ENABLE_I                PID_TRUE
#define SPEED_PID_ENABLE_D                PID_TRUE

/**
 * @brief 速度环比例系数
 *
 * @note 控制速度跟随目标值的能力
 *       初始值设为 0.0f，需要从 0 开始逐步增加调试
 *       典型范围: 0.1 ~ 5.0
 */
#define SPEED_KP                          (0.146f)

/**
 * @brief 速度环积分系数
 *
 * @note 消除速度稳态误差
 *       初始值设为 0.0f，根据调试效果调整
 *       典型范围: 0.01 ~ 1.0
 */
#define SPEED_KI                          (0.0f)

/**
 * @brief 速度环微分系数
 *
 * @note 预测速度变化趋势,抑制振荡
 *       初始值设为 0.0f，根据调试效果调整
 *       典型范围: 0.0 ~ 0.5
 */
#define SPEED_KD                          (0.0f)

/**
 * @brief 速度环输出限幅 (VMC 角度限制)
 *
 * @note 速度环PID输出直接作为VMC目标角度
 *       限制在 ±20° 范围内,防止姿态失控
 *       单位: 度 (°)
 */
//#define SPEED_OUT_MAX             (55.0f)
//#define SPEED_OUT_MIN             (-25.0f)
#define SPEED_OUT_MAX                     (35.0f)
#define SPEED_OUT_MIN                     (-25.0f)
/**
 * @brief 速度环积分限幅
 *
 * @note 防止积分饱和
 *       单位: 速度*时间累积量
 */
#define SPEED_PID_OUTPUT_MAX              (-SPEED_OUT_MIN)
#define SPEED_PID_OUTPUT_MIN              (-SPEED_OUT_MAX)
#define SPEED_PID_INTEGRAL_MAX            (5.0f)
#define SPEED_PID_INTEGRAL_MIN            (-5.0f)
#define SPEED_PID_I_STEP_MAX              (0.20f)
#define SPEED_PID_I_STEP_MIN              (-0.20f)
#define SPEED_PID_ANTI_WINDUP_ENABLE      PID_TRUE
#define SPEED_PID_D_MODE                  PID_DERIVATIVE_ON_MEASUREMENT
#define SPEED_PID_D_LPF_ALPHA             (0.6f)
#define SPEED_PID_RATE_LIMIT_ENABLE       PID_FALSE
#define SPEED_PID_RATE_UP                 (0.0f)
#define SPEED_PID_RATE_DOWN               (0.0f)

/**
 * @brief 固定车身高度 (VMC控制)
 *
 * @note 速度环控制时保持固定高度
 *       单位: cm (根据实际机械参数调整)
 */
#define FIXED_HEIGHT              (5.9f)

/* ========================================================================
 * 横滚角保护参数宏定义
 * ======================================================================== */

/**
 * @brief 横滚角死区阈值
 *
 * @note 两段控制分界线：
 *       |Roll| >= ROLL_DEAD_ZONE 使用全量 PID
 *       |Roll| <  ROLL_DEAD_ZONE 使用比例衰减 PID
 *       单位: 度 (°)
 */
#define ROLL_DEAD_ZONE          (1.5f)
#define ROLL_DEAD_ZONE_EXIT     (1.5f)

/**
 * @brief 横滚角显示/控制零偏补偿
 *
 * @note 由于机械结构引入固定偏差，统一对 roll 做 +2° 修正
 */
#define ROLL_DISPLAY_OFFSET     (2.0f)

/**
 * @brief 横滚角危险区阈值
 *
 * @note 当 |Roll| > ROLL_DANGER_ZONE 时触发系统停机保护
 *       单位: 度 (°)
 *
 * @warning 浮浮酱建议: 根据实际测试结果，可能需要调整为 25°~30°
 */
#define ROLL_DANGER_ZONE        (45.0f)

/**
 * @brief 横滚角基础高度
 *
 * @note 死区内或高侧腿保持的基础高度
 *       单位: cm
 */
#define ROLL_BASE_HEIGHT        (5.9f)

/**
 * @brief 横滚角最大伸展高度
 *
 * @note 低侧腿伸展的最大机械限制
 *       单位: cm
 */
#define ROLL_MAX_HEIGHT         (14.5f)

/**
 * @brief 横滚角 PID 控制参数
 *
 * @note 用于低侧腿的伸长控制
 *       初始值较小，需要通过实验调试
 */                       
#define ROLL_KP                 (0)    // 比例系数
#define ROLL_KI                 (0)    // 积分系数
#define ROLL_KD                 (0)    // 微分系数
#define ROLL_INT_MAX            (35.0f) // 积分限幅 (防风up)
#define ROLL_INT_STEP_MAX       (0.80f)// 单次积分步进限幅
#define ROLL_CONTROL_DT         (0.02f)// 横滚控制周期(20ms)
#define ROLL_INT_DECAY_FACTOR   (0.90f)// 退出区积分衰减系数
#define ROLL_RELEASE_HEIGHT_EPS (0.15f)// 退出补偿前高度接近阈值(cm)

/**
 * @brief 横滚角输出限幅
 *
 * @note 限制 PD 控制的输出范围
 *       单位: cm (高度增量)
 */
#define ROLL_OUT_MAX            (10.0f)   // 最大伸长量
#define ROLL_OUT_MIN            (0.0f)    // 最小伸长量 (不缩短)

/**
 * @brief 横滚角平滑滤波系数
 *
 * @note 一阶低通滤波: y[n] = α * x[n] + (1-α) * y[n-1]
 *       α = 0.3 表示新数据权重 30%，旧数据权重 70%
 */
#define ROLL_FILTER_ALPHA       (0.3f)

/* ========================================================================
 * 横滚角保护可调参数 (用于菜单调节)
 * ======================================================================== */

/**
 * @brief 横滚角 PID 控制参数 (可运行时调节)
 *
 * @note 这些参数可以通过菜单实时调节
 */
extern float g_Roll_Kp;    // 横滚角比例系数
extern float g_Roll_Ki;    // 横滚角积分系数
extern float g_Roll_Kd;    // 横滚角微分系数

/* ========================================================================
 * 系统状态定义
 * ======================================================================== */

/**
 * @brief 系统状态枚举
 */
typedef enum {
    SYSTEM_STATE_STOP = 0,         /**< 停机状态 (异常保护或未启动) */
    SYSTEM_STATE_RUN  = 1          /**< 运行状态 (正常平衡控制) */
} system_state_enum_t;

/* ========================================================================
 * PID控制器声明 (供外部调参使用)
 * ======================================================================== */

/**
 * @brief 外环 - 角度环 PID 控制器
 *
 * @note 用于外部串口调参
 */
extern pid_struct_t balance_angle_pid;

/**
 * @brief 内环 - 角速度环 PID 控制器
 *
 * @note 用于外部串口调参
 */
extern pid_struct_t balance_gyro_pid;

/**
 * @brief 速度环 PID 控制器
 *
 * @note 用于外部串口调参
 */
extern pid_struct_t speed_pid;

/* ========================================================================
 * 全局调试变量
 * ======================================================================== */

/**
 * @brief 当前俯仰角 (单位: 度)
 *
 * @note 用于外部串口任务观测
 */
extern float g_Debug_Pitch;

/**
 * @brief 当前角速度 (单位: °/s)
 *
 * @note 用于外部串口任务观测
 *       原始值较小，可能需要放大显示
 */
extern float g_Debug_Gyro;

/**
 * @brief 计算出的最终 PWM 输出
 *
 * @note 用于外部串口任务观测
 *       范围: -10000 ~ 10000
 */
extern int16_t g_Debug_PWM;

/**
 * @brief 系统状态 (0=STOP, 1=RUN)
 *
 * @note 用于外部串口任务观测和 LED 指示
 */
extern int8_t g_System_State;

/**
 * @brief 外环输出 - 目标角速度 (串级控制专用)
 *
 * @note 用于外部串口观测双环控制效果
 *       单位: °/s
 */
extern float g_Debug_Target_Gyro;

extern int16_t g_Debug_PWM_temp;
extern float mECHANICAL_ZERO;
extern int8_t speed_flag;

/* ========================================================================
 * 横滚角保护调试变量
 * ======================================================================== */

/**
 * @brief 当前横滚角 (单位: 度)
 *
 * @note 用于外部串口任务观测
 */
extern float g_Debug_Roll;

/**
 * @brief 当前横滚角显示值 (单位: 度, 已做零偏补偿)
 */
extern float g_Debug_Roll_Display;

/**
 * @brief 横滚角保护后左侧高度 (单位: cm)
 *
 * @note 用于外部串口任务观测
 */
extern float g_Debug_Height_L;

/**
 * @brief 横滚角保护后右侧高度 (单位: cm)
 *
 * @note 用于外部串口任务观测
 */
extern float g_Debug_Height_R;

/**
 * @brief 横滚角保护状态 (0=死区, 1=左侧代偿, 2=右侧代偿, 3=危险)
 *
 * @note 用于外部串口任务观测
 *       0 = 死区 (|Roll| ≤ 3°)
 *       1 = 左侧代偿 (Roll < 0°, 左腿伸长)
 *       2 = 右侧代偿 (Roll > 0°, 右腿伸长)
 *       3 = 危险区 (|Roll| > 45°)
 */
extern int8_t g_Debug_Roll_State;

// ============================================================
// 速度环调试变量
// ============================================================

/**
 * @brief 速度环目标速度 (可由外部遥控或按键设置)
 *
 * @note 单位: 与电机速度反馈相同单位
 *       正值=前进, 负值=后退
 */
extern float g_Debug_Target_Speed;

/**
 * @brief 速度环测量速度 (滤波后的平均速度)
 *
 * @note 用于观测速度反馈
 *       单位: 与电机速度反馈相同单位
 */
extern float g_Debug_Measure_Speed;

/**
 * @brief 速度环PID输出 (VMC目标角度)
 *
 * @note 用于观测速度环控制效果
 *       单位: 度 (°)
 */
extern float g_Debug_Speed_Output;

/* ========================================================================
 * 函数声明
 * ======================================================================== */

/**
 * @brief 控制系统初始化
 *
 * @note 功能说明:
 *       1. 初始化直立环 PID 控制器
 *       2. 设置机械零点和保护阈值
 *       3. 初始化系统状态为 STOP
 *
 * @note 使用示例:
 *       在主函数中调用一次:
 *       control_init();
 */
void control_init(void);

/**
 * @brief 核心控制任务 (1ms 周期调用)
 *
 * @note 功能说明:
 *       1. 异常保护检测 (角度超限 → 停机)
 *       2. 自动恢复判断 (角度回归 → 恢复运行)
 *       3. 直立环 PD 计算
 *       4. 电机混合输出 (考虑极性)
 *       5. 舵机中位锁死 (保持腿部刚度)
 *
 * @note 使用示例:
 *       在 1ms 定时器中断中调用:
 *       void pit_handler(void) {
 *           control_run_1ms();
 *       }
 */
void control_run_1ms(void);

/**
 * @brief 系统状态控制 (手动启停)
 *
 * @param state 目标状态 (0=STOP, 1=RUN)
 *
 * @note 使用场景:
 *       - 按键启停控制
 *       - 遥控器启停控制
 *       - 调试时的手动控制
 */
void control_set_state(uint8_t state);

/**
 * @brief 速度环控制任务 (20ms周期调用)
 *
 * @note 功能说明:
 *       1. 读取左右轮速度并滤波
 *       2. 计算速度环PID输出 → VMC目标角度
 *       3. 调用VMC更新所有舵机角度
 *       4. 在非运行状态重置PID积分
 *
 * @note 使用示例:
 *       在20ms定时任务中调用:
 *       if (uwtick % 20 == 0) {
 *           speed_control_run();
 *       }
 */
void speed_control_run(void);

/**
 * @brief 横滚角防护计算函数 (20ms 周期调用)
 *
 * @param roll_current 当前横滚角 (单位: 度)
 * @param height_L 输出: 左侧腿高度 (单位: cm)
 * @param height_R 输出: 右侧腿高度 (单位: cm)
 *
 * @note 功能说明:
 *       1. 死区保护: |Roll| ≤ 3° → 左右腿均保持 4.4cm
 *       2. 动态代偿: 3° < |Roll| ≤ 45° → 单边伸长补偿
 *          - Roll < 0° (左侧偏低) → 左腿伸长，右腿保持 4.4cm
 *          - Roll > 0° (右侧偏高) → 右腿伸长，左腿保持 4.4cm
 *       3. 平滑滤波: 防止高度突变
 *
 * @note 使用示例:
 *       float height_l, height_r;
 *       Roll_Protection_Compute(icm_output.roll, &height_l, &height_r);
 *       VMC_Update_All_Servos_Roll(height_r, height_l, speed_angle);
 */
void Roll_Protection_Compute(float roll_current, float* height_L, float* height_R);

#endif /* CODE_CONTROL_H_ */

/**
 * *****************************************************************************
 * @file    pid.h
 * @brief   通用 PID 控制器模块头文件
 * @author  浮浮酱 (幽浮喵)
 * @date    2025-01-31
 * @version 1.0
 *
 * ==============================================================================
 * 模块说明:
 *     提供标准的 PID 控制算法，支持 P、I、D 项独立配置
 *     适用于平衡小车的直立环、速度环、转向环等多种控制场景
 *
 * 核心特性:
 *     1. 支持 PI/PD/PID 全模式
 *     2. 积分限幅防止积分饱和
 *     3. 输出限幅保护执行机构
 *     4. 微分项可选测量值微分或误差微分
 * ==============================================================================
 *
 * *****************************************************************************
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"  // 只包含必要的底层头文件

/* ========================================================================
 * PID 结构体定义
 * ======================================================================== */

/**
 * @brief PID 控制器结构体
 *
 * @note 使用说明:
 *       1. 初始化时设置 kp, ki, kd 系数
 *       2. 根据需要设置积分限幅和输出限幅
 *       3. 调用 pid_compute() 进行周期计算
 */
typedef struct {
    float kp;             /**< 比例系数 Proportional Gain */
    float ki;             /**< 积分系数 Integral Gain */
    float kd;             /**< 微分系数 Derivative Gain */

    float target;         /**< 目标值 (设定点) Setpoint */
    float measure;        /**< 测量值 (反馈值) Measured Value */

    float error;          /**< 当前误差 Error = Target - Measure */
    float last_error;     /**< 上次误差 Last Error (用于微分计算) */
    float error_sum;      /**< 误差累积 Integral Sum */

    float output;         /**< PID 计算输出 Output */

    float integral_max;   /**< 积分限幅上限 (防止积分饱和) */
    float integral_min;   /**< 积分限幅下限 */
    float output_max;     /**< 输出限幅上限 (保护执行机构) */
    float output_min;     /**< 输出限幅下限 */

    uint8_t first_cycle;  /**< 首次循环标志 (用于微分初始化) */
} pid_struct_t;

/* ========================================================================
 * 函数声明
 * ======================================================================== */

/**
 * @brief PID 控制器初始化
 *
 * @param pid          PID 结构体指针
 * @param kp           比例系数
 * @param ki           积分系数
 * @param kd           微分系数
 * @param integral_max 积分限幅 (设为 0 时不限幅)
 * @param output_max   输出限幅上限
 * @param output_min   输出限幅下限
 *
 * @note 使用示例:
 *       pid_struct_t balance_pid;
 *       pid_init(&balance_pid, 0.0f, 0.0f, 0.0f, 1000.0f, 10000.0f, -10000.0f);
 */
void pid_init(pid_struct_t *pid,
              float kp, float ki, float kd,
              float integral_max,
              float output_max, float output_min);

/**
 * @brief PID 计算 (核心算法)
 *
 * @param pid    PID 结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return float PID 计算输出
 *
 * @note 算法说明:
 *       Output = Kp*Error + Ki*∫Error*dt + Kd*d(Error)/dt
 *       微分项采用误差微分 (Error - Last_Error)
 *
 *       使用示例:
 *       float output = pid_compute(&balance_pid, -10.6f, current_pitch);
 */
float pid_compute(pid_struct_t *pid, float target, float measure);

/**
 * @brief 重置 PID 状态 (清除累积误差)
 *
 * @param pid PID 结构体指针
 *
 * @note 适用场景:
 *       - 控制模式切换时
 *       - 设定点大幅变化时
 *       - 系统重启或异常恢复后
 */
void pid_reset(pid_struct_t *pid);

/**
 * @brief 设置 PID 目标值
 *
 * @param pid    PID 结构体指针
 * @param target 目标值
 */
void pid_set_target(pid_struct_t *pid, float target);

/**
 * @brief 设置 PID 系数 (运行时调整)
 *
 * @param pid PID 结构体指针
 * @param kp  比例系数
 * @param ki  积分系数
 * @param kd  微分系数
 */
void pid_set_coefficients(pid_struct_t *pid, float kp, float ki, float kd);

#endif /* CODE_PID_H_ */

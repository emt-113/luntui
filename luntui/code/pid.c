/**
 * *****************************************************************************
 * @file    pid.c
 * @brief   通用 PID 控制器模块实现
 * @author  浮浮酱 (幽浮喵)
 * @date    2025-01-31
 * @version 1.0
 *
 * *****************************************************************************
 */

#include "pid.h"
#include <string.h>

/* ========================================================================
 * 函数实现
 * ======================================================================== */

/**
 * @brief PID 控制器初始化
 */
void pid_init(pid_struct_t *pid,
              float kp, float ki, float kd,
              float integral_max,
              float output_max, float output_min)
{
    if (pid == NULL) return;

    // 清空结构体
    memset(pid, 0, sizeof(pid_struct_t));

    // 设置 PID 系数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    // 设置限幅
    pid->integral_max = integral_max;
    pid->integral_min = -integral_max;  // 对称限幅
    pid->output_max = output_max;
    pid->output_min = output_min;

    // 初始化状态
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->error_sum = 0.0f;
    pid->output = 0.0f;
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->first_cycle = 1;  // 标记为首次循环
}

/**
 * @brief PID 计算 (核心算法)
 *
 * 算法公式:
 *     Output = Kp * Error(t) + Ki * ∑Error(t) * dt + Kd * (Error(t) - Error(t-1)) / dt
 *
 * 注意事项:
 *     1. 微分项采用误差微分 (对设定点变化敏感)
 *     2. 积分项带限幅防止积分饱和
 *     3. 输出限幅保护执行机构
 */
float pid_compute(pid_struct_t *pid, float target, float measure)
{
    if (pid == NULL) return 0.0f;

    // 更新目标值和测量值
    pid->target = target;
    pid->measure = measure;

    // 计算当前误差
    pid->error = target - measure;

    // ============================================================
    // 比例项 (Proportional): Kp * Error
    // ============================================================
    float p_out = pid->kp * pid->error;

    // ============================================================
    // 积分项 (Integral): Ki * ∑Error
    // ============================================================
    float i_out = 0.0f;

    if (pid->ki != 0.0f)
    {
        // 累积误差
        pid->error_sum += pid->error;

        // 积分限幅 (防止积分饱和)
        if (pid->integral_max != 0.0f)
        {
            if (pid->error_sum > pid->integral_max)
            {
                pid->error_sum = pid->integral_max;
            }
            else if (pid->error_sum < pid->integral_min)
            {
                pid->error_sum = pid->integral_min;
            }
        }

        i_out = pid->ki * pid->error_sum;
    }

    // ============================================================
    // 微分项 (Derivative): Kd * d(Error)/dt
    // ============================================================
    float d_out = 0.0f;

    if (pid->kd != 0.0f)
    {
        if (pid->first_cycle)
        {
            // 首次循环，微分项为 0
            pid->last_error = pid->error;
            pid->first_cycle = 0;
        }
        else
        {
            // 误差微分法 (对设定点变化敏感)
            d_out = pid->kd * (pid->error - pid->last_error);
        }

        // 更新上次误差
        pid->last_error = pid->error;
    }

    // ============================================================
    // 总输出计算
    // ============================================================
    pid->output = p_out + i_out + d_out;

    // 输出限幅
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }

    return pid->output;
}

/**
 * @brief 重置 PID 状态
 */
void pid_reset(pid_struct_t *pid)
{
    if (pid == NULL) return;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->error_sum = 0.0f;
    pid->output = 0.0f;
    pid->first_cycle = 1;
}

/**
 * @brief 设置 PID 目标值
 */
void pid_set_target(pid_struct_t *pid, float target)
{
    if (pid == NULL) return;
    pid->target = target;
}

/**
 * @brief 设置 PID 系数 (运行时调整)
 */
void pid_set_coefficients(pid_struct_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

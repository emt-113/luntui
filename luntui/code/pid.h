/**
 * *****************************************************************************
 * @file    pid.h
 * @brief   通用 PID 控制器模块头文件
 * @author  浮浮酱 (幽浮喵)
 * @date    2026-03-22
 * @version 2.0
 *
 * *****************************************************************************
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

/* ========================================================================
 * 常量定义
 * ======================================================================== */

#define PID_FALSE                       (0U)
#define PID_TRUE                        (1U)

#define PID_DERIVATIVE_ON_ERROR         (0U)
#define PID_DERIVATIVE_ON_MEASUREMENT   (1U)

#define PID_LIMIT_NONE                  (1.0e30f)

/* ========================================================================
 * PID 结构体定义
 * ======================================================================== */

typedef struct {
    float kp;
    float ki;
    float kd;

    float target;
    float measure;

    float error;
    float last_error;

    float integral;
    float integral_max;
    float integral_min;
    float integral_step_max;
    float integral_step_min;

    float derivative;
    float derivative_raw;
    float d_lpf_alpha;
    float last_measure;

    float p_out;
    float i_out;
    float d_out;
    float output;
    float last_output;

    float output_max;
    float output_min;

    float output_rate_up;
    float output_rate_down;

    uint8_t enable_p;
    uint8_t enable_i;
    uint8_t enable_d;
    uint8_t anti_windup_enable;
    uint8_t derivative_mode;
    uint8_t output_rate_limit_enable;
    uint8_t first_cycle;
} pid_struct_t;

/* ========================================================================
 * 函数声明
 * ======================================================================== */

void pid_init(pid_struct_t *pid,
              float kp, float ki, float kd,
              float integral_max,
              float output_max, float output_min);

void pid_reset(pid_struct_t *pid);

float pid_compute(pid_struct_t *pid, float target, float measure, float dt_s);

void pid_set_target(pid_struct_t *pid, float target);
void pid_set_coefficients(pid_struct_t *pid, float kp, float ki, float kd);

void pid_set_enable(pid_struct_t *pid, uint8_t enable_p, uint8_t enable_i, uint8_t enable_d);
void pid_set_output_limit(pid_struct_t *pid, float output_min, float output_max);
void pid_set_integral_limit(pid_struct_t *pid, float integral_min, float integral_max);
void pid_set_integral_step_limit(pid_struct_t *pid, float integral_step_min, float integral_step_max);
void pid_set_anti_windup(pid_struct_t *pid, uint8_t enable);
void pid_set_derivative_mode(pid_struct_t *pid, uint8_t derivative_mode);
void pid_set_derivative_lpf(pid_struct_t *pid, float alpha);
void pid_set_output_rate_limit(pid_struct_t *pid,
                               uint8_t enable,
                               float rate_down,
                               float rate_up);

float pid_apply_rate_limit(float target,
                           float last_output,
                           float rate_down,
                           float rate_up,
                           float dt_s);

#endif /* CODE_PID_H_ */

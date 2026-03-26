/**
 * *****************************************************************************
 * @file    pid.c
 * @brief   通用 PID 控制器模块实现
 * @author  浮浮酱 (幽浮喵)
 * @date    2026-03-22
 * @version 2.0
 *
 * *****************************************************************************
 */

#include "pid.h"
#include <string.h>

static float pid_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

float pid_apply_rate_limit(float target,
                           float last_output,
                           float rate_down,
                           float rate_up,
                           float dt_s)
{
    float delta;
    float delta_max_up;
    float delta_max_down;

    if (dt_s <= 0.0f)
    {
        return target;
    }

    delta = target - last_output;
    delta_max_up = rate_up * dt_s;
    delta_max_down = rate_down * dt_s;

    if (delta > delta_max_up)
    {
        delta = delta_max_up;
    }
    else if (delta < -delta_max_down)
    {
        delta = -delta_max_down;
    }

    return last_output + delta;
}

void pid_init(pid_struct_t *pid,
              float kp, float ki, float kd,
              float integral_max,
              float output_max, float output_min)
{
    if (pid == NULL)
    {
        return;
    }

    memset(pid, 0, sizeof(pid_struct_t));

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->enable_p = PID_TRUE;
    pid->enable_i = (ki != 0.0f) ? PID_TRUE : PID_FALSE;
    pid->enable_d = (kd != 0.0f) ? PID_TRUE : PID_FALSE;

    pid->anti_windup_enable = PID_TRUE;
    pid->derivative_mode = PID_DERIVATIVE_ON_MEASUREMENT;
    pid->d_lpf_alpha = 1.0f;

    if (integral_max > 0.0f)
    {
        pid->integral_max = integral_max;
        pid->integral_min = -integral_max;
        pid->integral_step_max = integral_max;
        pid->integral_step_min = -integral_max;
    }
    else
    {
        pid->integral_max = PID_LIMIT_NONE;
        pid->integral_min = -PID_LIMIT_NONE;
        pid->integral_step_max = PID_LIMIT_NONE;
        pid->integral_step_min = -PID_LIMIT_NONE;
    }

    pid->output_max = output_max;
    pid->output_min = output_min;

    pid->output_rate_limit_enable = PID_FALSE;
    pid->output_rate_up = 0.0f;
    pid->output_rate_down = 0.0f;

    pid->first_cycle = PID_TRUE;
}

void pid_reset(pid_struct_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->derivative_raw = 0.0f;
    pid->last_measure = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;
    pid->first_cycle = PID_TRUE;
}

float pid_compute(pid_struct_t *pid, float target, float measure, float dt_s)
{
    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;
    float output_unsat;
    float output_sat;
    float integral_step = 0.0f;
    float integral_candidate;
    float derivative_raw = 0.0f;
    uint8_t allow_integral = PID_TRUE;

    if ((pid == NULL) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    pid->target = target;
    pid->measure = measure;
    pid->error = target - measure;

    if (pid->first_cycle == PID_TRUE)
    {
        pid->last_error = pid->error;
        pid->last_measure = pid->measure;
        pid->derivative = 0.0f;
        pid->derivative_raw = 0.0f;
        pid->last_output = pid->output;
        pid->first_cycle = PID_FALSE;
    }

    if (pid->enable_p == PID_TRUE)
    {
        p_out = pid->kp * pid->error;
    }

    if (pid->enable_d == PID_TRUE)
    {
        if (pid->derivative_mode == PID_DERIVATIVE_ON_MEASUREMENT)
        {
            derivative_raw = -(pid->measure - pid->last_measure) / dt_s;
        }
        else
        {
            derivative_raw = (pid->error - pid->last_error) / dt_s;
        }

        pid->derivative_raw = derivative_raw;
        pid->derivative += pid->d_lpf_alpha * (derivative_raw - pid->derivative);
        d_out = pid->kd * pid->derivative;
    }

    if (pid->enable_i == PID_TRUE)
    {
        integral_step = pid->error * dt_s;
        integral_step = pid_clamp(integral_step,
                                  pid->integral_step_min,
                                  pid->integral_step_max);

        integral_candidate = pid->integral + integral_step;
        integral_candidate = pid_clamp(integral_candidate,
                                       pid->integral_min,
                                       pid->integral_max);

        if (pid->anti_windup_enable == PID_TRUE)
        {
            output_unsat = p_out + pid->ki * integral_candidate + d_out;

            if ((output_unsat > pid->output_max) && (pid->error > 0.0f))
            {
                allow_integral = PID_FALSE;
            }
            else if ((output_unsat < pid->output_min) && (pid->error < 0.0f))
            {
                allow_integral = PID_FALSE;
            }
        }

        if (allow_integral == PID_TRUE)
        {
            pid->integral = integral_candidate;
        }

        i_out = pid->ki * pid->integral;
    }

    output_unsat = p_out + i_out + d_out;
    output_sat = pid_clamp(output_unsat, pid->output_min, pid->output_max);

    if (pid->output_rate_limit_enable == PID_TRUE)
    {
        output_sat = pid_apply_rate_limit(output_sat,
                                          pid->last_output,
                                          pid->output_rate_down,
                                          pid->output_rate_up,
                                          dt_s);
        output_sat = pid_clamp(output_sat, pid->output_min, pid->output_max);
    }

    pid->p_out = p_out;
    pid->i_out = i_out;
    pid->d_out = d_out;
    pid->output = output_sat;
    pid->last_output = output_sat;
    pid->last_error = pid->error;
    pid->last_measure = pid->measure;

    return pid->output;
}

void pid_set_target(pid_struct_t *pid, float target)
{
    if (pid == NULL)
    {
        return;
    }

    pid->target = target;
}

void pid_set_coefficients(pid_struct_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->enable_i = (ki != 0.0f) ? PID_TRUE : PID_FALSE;
    pid->enable_d = (kd != 0.0f) ? PID_TRUE : PID_FALSE;
}

void pid_set_enable(pid_struct_t *pid, uint8_t enable_p, uint8_t enable_i, uint8_t enable_d)
{
    if (pid == NULL)
    {
        return;
    }

    pid->enable_p = enable_p;
    pid->enable_i = enable_i;
    pid->enable_d = enable_d;
}

void pid_set_output_limit(pid_struct_t *pid, float output_min, float output_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->output_min = output_min;
    pid->output_max = output_max;
}

void pid_set_integral_limit(pid_struct_t *pid, float integral_min, float integral_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = pid_clamp(pid->integral, pid->integral_min, pid->integral_max);
}

void pid_set_integral_step_limit(pid_struct_t *pid, float integral_step_min, float integral_step_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral_step_min = integral_step_min;
    pid->integral_step_max = integral_step_max;
}

void pid_set_anti_windup(pid_struct_t *pid, uint8_t enable)
{
    if (pid == NULL)
    {
        return;
    }

    pid->anti_windup_enable = enable;
}

void pid_set_derivative_mode(pid_struct_t *pid, uint8_t derivative_mode)
{
    if (pid == NULL)
    {
        return;
    }

    pid->derivative_mode = derivative_mode;
}

void pid_set_derivative_lpf(pid_struct_t *pid, float alpha)
{
    if (pid == NULL)
    {
        return;
    }

    pid->d_lpf_alpha = pid_clamp(alpha, 0.0f, 1.0f);
}

void pid_set_output_rate_limit(pid_struct_t *pid,
                               uint8_t enable,
                               float rate_down,
                               float rate_up)
{
    if (pid == NULL)
    {
        return;
    }

    pid->output_rate_limit_enable = enable;
    pid->output_rate_down = rate_down;
    pid->output_rate_up = rate_up;
}

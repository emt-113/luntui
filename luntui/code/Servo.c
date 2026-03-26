/*
 * Servo.c
 *
 *  Created on: 2025��3��1��
 *      Author: 26766
 */
#include "Servo.h"
#include "math.h"
int16_t chushixiajiang  = 0  ; //+��ʾС���߶����ߣ�-��ʾ�½�
extern uint8_t run_flag;
serv_control servo_1;
serv_control servo_2;
serv_control servo_3;
serv_control servo_4;

int jump_flag = 0;
void Servo_Init(void)
{
    servo_1.servo_pin = Servo_1;
    servo_1.fre=300;
    servo_1.center= servo_1_center;
    servo_1.dir = servo_1_dir;

    servo_2.servo_pin = Servo_2;
    servo_2.fre=300;
    servo_2.center= servo_2_center;
    servo_2.dir = servo_2_dir;

    servo_3.servo_pin = Servo_3;
    servo_3.fre=300;
    servo_3.center= servo_3_center;
    servo_3.dir = servo_3_dir;

    servo_4.servo_pin = Servo_4;
    servo_4.fre=300;
    servo_4.center= servo_4_center;
    servo_4.dir = servo_4_dir;

    servo_1.center = servo_1.center - chushixiajiang * servo_1.dir;
    servo_2.center = servo_2.center - chushixiajiang * servo_2.dir;
    servo_3.center = servo_3.center - chushixiajiang * servo_3.dir;
    servo_4.center = servo_4.center - chushixiajiang * servo_4.dir;

    servo_1.now_location = servo_1.center;
    servo_2.now_location = servo_2.center;
    servo_3.now_location = servo_3.center;
    servo_4.now_location = servo_4.center;


    pwm_init(servo_1.servo_pin, servo_1.fre, servo_1.now_location);
    pwm_init(servo_2.servo_pin, servo_2.fre, servo_2.now_location);
    pwm_init(servo_3.servo_pin, servo_3.fre, servo_3.now_location);
    pwm_init(servo_4.servo_pin, servo_4.fre, servo_4.now_location);

}
int16 xianfu(int16 pwm, int16_t max, int16_t min)
{
    if (pwm > max) pwm = max;
    if (pwm < min) pwm = min;
    else
    {
        return pwm;
    }
    return pwm;
}
void set_servo_duty(serv_control *servo, int16 duty)
{
    servo->now_location = xianfu(duty, 10000, -10000);
    pwm_set_duty(servo->servo_pin, duty);
}

//void servo_control_table(float p, float angle, int16_t* pwm1, int16_t* pwm2);
float gao=4.5,angle=0;
float gao_=4.5,angle_=40;
void shi()
{
   int16_t offset1, offset2;
    int32_t final_pwm1, final_pwm2;
    printf("Input: gao=%f, angle=%f\r\n", gao, angle);
    // 1. ���ò������
    servo_control_table(gao, angle, &offset1, &offset2);
    printf("%d,%d\r\n",offset1,offset2);
    // 2. �����Ч��
    if (offset1 == 10000 || offset2 == 10000) {
        // ����������ֹͣ����򱣳���һ�ε�λ�ã���ֹԽ��
        return; 
    }

    // 3. ������ֵ
//    final_pwm1 = servo_1.center + offset1* servo_1.dir;
//    final_pwm2 = servo_2.center + offset2* servo_2.dir;
   final_pwm1 = servo_3.center + offset1* servo_3.dir;
   final_pwm2 = servo_4.center + offset2* servo_4.dir;
    // 4. �޷�������180�ȶ��ͨ����Ӧ 0.5ms - 2.5ms��
    // 0.5ms ��Ӧ 1500, 2.5ms ��Ӧ 7500 (ARR=10000ʱ)
    if (final_pwm1 < 500) final_pwm1 = 500;
    if (final_pwm1 > 9500) final_pwm1 = 9500;

    // 5. д��Ӳ���Ĵ���
    set_servo_duty(&servo_3, final_pwm1);
    set_servo_duty(&servo_4, final_pwm2);
}


/*
 * 函数名: VMC_Update_All_Servos
 * 功能: 根据目标高度和角度，更新所有4个舵机的PWM值，带防抖处理
 * 参数:
 *   gao: 目标高度（单位：cm）
 *   angle: 目标角度（单位：度，+20表示前倾，-20表示后倾）
 *
 * 防抖策略说明:
 *   不使用低通滤波（会有延迟），而是使用PWM变化率限制（Slew Rate Limit）
 *   定义MAX_PWM_STEP宏，限制每次PWM变化的最大步长
 *   使用static数组记忆上一次的PWM值，实现平滑过渡
 */
int8_t MAX_PWM_STEP = 55 ; // PWM每帧最大变化量，可根据实际效果调整（推荐值：100-200）//45

void VMC_Update_All_Servos(float gao, float angle)
{
    // 1. 输入限幅处理（将angle限制在[-30, 30]之间）
    if(angle > 30.0f) angle = 30.0f;
    if(angle < -30.0f) angle = -30.0f;

    // 2. 查表获取4个舵机的偏移量
    //    注意：查表函数已在vmc.c中实现，这里直接调用
    int16_t offset1, offset2, offset3, offset4;
    servo_control_table(gao, angle, &offset1, &offset2);  // 舵机1和2的偏移
    servo_control_table(gao, angle, &offset3, &offset4);  // 舵机3和4的偏移

    // 3. 查表有效性检查（如果返回10000表示超出查表范围）
    if(offset1 == 10000 || offset2 == 10000 || offset3 == 10000 || offset4 == 10000)
    {
        // 超出有效范围，保持当前位置不变，防止舵机误动作
        return;
    }

    // 4. 计算目标PWM值
    //    逻辑：Target_PWM = Center + Offset * Direction
    //    Angle=+20（前倾） -> Offset为正 -> PWM增大或减小取决于dir
    int32_t target_pwm1 = servo_1.center + offset1 * servo_1.dir;
    int32_t target_pwm2 = servo_2.center + offset2 * servo_2.dir;
    int32_t target_pwm3 = servo_3.center + offset3 * servo_3.dir;
    int32_t target_pwm4 = servo_4.center + offset4 * servo_4.dir;

    // 5. 【核心步骤】PWM变化率限制（Slew Rate Limit）防抖处理
    //    使用static变量记忆上一次的PWM值，实现平滑过渡
    static int32_t last_pwm1 = 0, last_pwm2 = 0, last_pwm3 = 0, last_pwm4 = 0;

    //    初始化（第一次调用时使用当前值）
    if(last_pwm1 == 0) last_pwm1 = servo_1.now_location;
    if(last_pwm2 == 0) last_pwm2 = servo_2.now_location;
    if(last_pwm3 == 0) last_pwm3 = servo_3.now_location;
    if(last_pwm4 == 0) last_pwm4 = servo_4.now_location;

    //    变化率限制逻辑：
    //    如果目标值与上一次值的差值超过MAX_PWM_STEP，
    //    则当前输出只能在"上一次值"的基础上增加或减少MAX_PWM_STEP
    //    这样可以有效防止舵机猛烈动作，避免机械冲击

    // 舵机1变化率限制
    int32_t pwm1 = last_pwm1;
    if(target_pwm1 - last_pwm1 > MAX_PWM_STEP)
        pwm1 = last_pwm1 + MAX_PWM_STEP;  // 目标过大，只能增加MAX_PWM_STEP
    else if(last_pwm1 - target_pwm1 > MAX_PWM_STEP)
        pwm1 = last_pwm1 - MAX_PWM_STEP;  // 目标过小，只能减少MAX_PWM_STEP
    else
        pwm1 = target_pwm1;               // 在允许范围内，直接使用目标值

    // 舵机2变化率限制
    int32_t pwm2 = last_pwm2;
    if(target_pwm2 - last_pwm2 > MAX_PWM_STEP)
        pwm2 = last_pwm2 + MAX_PWM_STEP;
    else if(last_pwm2 - target_pwm2 > MAX_PWM_STEP)
        pwm2 = last_pwm2 - MAX_PWM_STEP;
    else
        pwm2 = target_pwm2;

    // 舵机3变化率限制
    int32_t pwm3 = last_pwm3;
    if(target_pwm3 - last_pwm3 > MAX_PWM_STEP)
        pwm3 = last_pwm3 + MAX_PWM_STEP;
    else if(last_pwm3 - target_pwm3 > MAX_PWM_STEP)
        pwm3 = last_pwm3 - MAX_PWM_STEP;
    else
        pwm3 = target_pwm3;

    // 舵机4变化率限制
    int32_t pwm4 = last_pwm4;
    if(target_pwm4 - last_pwm4 > MAX_PWM_STEP)
        pwm4 = last_pwm4 + MAX_PWM_STEP;
    else if(last_pwm4 - target_pwm4 > MAX_PWM_STEP)
        pwm4 = last_pwm4 - MAX_PWM_STEP;
    else
        pwm4 = target_pwm4;

    // 更新记忆值
    last_pwm1 = pwm1;
    last_pwm2 = pwm2;
    last_pwm3 = pwm3;
    last_pwm4 = pwm4;

    // 6. 绝对安全限幅（将PWM值限制在[500, 9500]之间）
    //    这是硬件安全范围，防止PWM值过大或过小损坏舵机
    if(pwm1 < 500) pwm1 = 500;
    if(pwm1 > 9500) pwm1 = 9500;

    if(pwm2 < 500) pwm2 = 500;
    if(pwm2 > 9500) pwm2 = 9500;

    if(pwm3 < 500) pwm3 = 500;
    if(pwm3 > 9500) pwm3 = 9500;

    if(pwm4 < 500) pwm4 = 500;
    if(pwm4 > 9500) pwm4 = 9500;

    // 7. 写入硬件寄存器
    set_servo_duty(&servo_1, (int16)pwm1);
    set_servo_duty(&servo_2, (int16)pwm2);
    set_servo_duty(&servo_3, (int16)pwm3);
    set_servo_duty(&servo_4, (int16)pwm4);
}

/*
 * 函数名: VMC_Update_All_Servos_Roll
 * 功能: 根据左右腿独立高度和俯仰角，更新所有4个舵机的PWM值，带防抖处理
 * 参数:
 *   height_R: 右侧腿目标高度（单位：cm）- 用于 Servo_1, Servo_2
 *   height_L: 左侧腿目标高度（单位：cm）- 用于 Servo_3, Servo_4
 *   angle: 俯仰角（单位：度，+20表示前倾，-20表示后倾）
 *
 * 防抖策略说明:
 *   不使用低通滤波（会有延迟），而是使用PWM变化率限制（Slew Rate Limit）
 *   定义MAX_PWM_STEP宏，限制每次PWM变化的最大步长
 *   使用static数组记忆上一次的PWM值，实现平滑过渡
 *
 * 舵机映射关系:
 *   - Servo_1, Servo_2: 右腿 (前后) → 使用 height_R
 *   - Servo_3, Servo_4: 左腿 (前后) → 使用 height_L
 */
void VMC_Update_All_Servos_Roll(float height_R, float height_L, float angle)
{
    // 1. 输入限幅处理（将angle限制在[-30, 30]之间）
    if(angle > 30.0f) angle = 30.0f;
    if(angle < -30.0f) angle = -30.0f;

    // 高度限幅
    if(height_L > 14.5f) height_L = 14.5f;
    if(height_L < 4.4f) height_L = 4.4f;
    if(height_R > 14.5f) height_R = 14.5f;
    if(height_R < 4.4f) height_R = 4.4f;

    // 2. 查表获取4个舵机的偏移量
    int16_t offset1_R, offset2_R;  // 右腿偏移量
    int16_t offset3_L, offset4_L;  // 左腿偏移量

    // 右腿 (Servo_1, Servo_2) 使用 height_R
    servo_control_table(height_R, angle, &offset1_R, &offset2_R);

    // 左腿 (Servo_3, Servo_4) 使用 height_L
    servo_control_table(height_L, angle, &offset3_L, &offset4_L);

    // 3. 查表有效性检查（如果返回10000表示超出查表范围）
    if(offset1_R == 10000 || offset2_R == 10000 ||
       offset3_L == 10000 || offset4_L == 10000)
    {
        // 超出有效范围，保持当前位置不变，防止舵机误动作
        return;
    }

    // 4. 计算目标PWM值
    int32_t target_pwm1 = servo_1.center + offset1_R * servo_1.dir;  // 右腿
    int32_t target_pwm2 = servo_2.center + offset2_R * servo_2.dir;  // 右腿
    int32_t target_pwm3 = servo_3.center + offset3_L * servo_3.dir;  // 左腿
    int32_t target_pwm4 = servo_4.center + offset4_L * servo_4.dir;  // 左腿

    // 5. PWM变化率限制（Slew Rate Limit）防抖处理
    static int32_t last_pwm1_roll = 0, last_pwm2_roll = 0, last_pwm3_roll = 0, last_pwm4_roll = 0;

    // 初始化（第一次调用时使用当前值）
    if(last_pwm1_roll == 0) last_pwm1_roll = servo_1.now_location;
    if(last_pwm2_roll == 0) last_pwm2_roll = servo_2.now_location;
    if(last_pwm3_roll == 0) last_pwm3_roll = servo_3.now_location;
    if(last_pwm4_roll == 0) last_pwm4_roll = servo_4.now_location;

    // 舵机1变化率限制
    int32_t pwm1 = last_pwm1_roll;
    if(target_pwm1 - last_pwm1_roll > MAX_PWM_STEP)
        pwm1 = last_pwm1_roll + MAX_PWM_STEP;
    else if(last_pwm1_roll - target_pwm1 > MAX_PWM_STEP)
        pwm1 = last_pwm1_roll - MAX_PWM_STEP;
    else
        pwm1 = target_pwm1;

    // 舵机2变化率限制
    int32_t pwm2 = last_pwm2_roll;
    if(target_pwm2 - last_pwm2_roll > MAX_PWM_STEP)
        pwm2 = last_pwm2_roll + MAX_PWM_STEP;
    else if(last_pwm2_roll - target_pwm2 > MAX_PWM_STEP)
        pwm2 = last_pwm2_roll - MAX_PWM_STEP;
    else
        pwm2 = target_pwm2;

    // 舵机3变化率限制
    int32_t pwm3 = last_pwm3_roll;
    if(target_pwm3 - last_pwm3_roll > MAX_PWM_STEP)
        pwm3 = last_pwm3_roll + MAX_PWM_STEP;
    else if(last_pwm3_roll - target_pwm3 > MAX_PWM_STEP)
        pwm3 = last_pwm3_roll - MAX_PWM_STEP;
    else
        pwm3 = target_pwm3;

    // 舵机4变化率限制
    int32_t pwm4 = last_pwm4_roll;
    if(target_pwm4 - last_pwm4_roll > MAX_PWM_STEP)
        pwm4 = last_pwm4_roll + MAX_PWM_STEP;
    else if(last_pwm4_roll - target_pwm4 > MAX_PWM_STEP)
        pwm4 = last_pwm4_roll - MAX_PWM_STEP;
    else
        pwm4 = target_pwm4;

    // 更新记忆值
    last_pwm1_roll = pwm1;
    last_pwm2_roll = pwm2;
    last_pwm3_roll = pwm3;
    last_pwm4_roll = pwm4;

    // 6. 绝对安全限幅（将PWM值限制在[500, 9500]之间）
    if(pwm1 < 500) pwm1 = 500;
    if(pwm1 > 9500) pwm1 = 9500;
    if(pwm2 < 500) pwm2 = 500;
    if(pwm2 > 9500) pwm2 = 9500;
    if(pwm3 < 500) pwm3 = 500;
    if(pwm3 > 9500) pwm3 = 9500;
    if(pwm4 < 500) pwm4 = 500;
    if(pwm4 > 9500) pwm4 = 9500;

    // 7. 写入硬件寄存器
    set_servo_duty(&servo_1, (int16)pwm1);
    set_servo_duty(&servo_2, (int16)pwm2);
    set_servo_duty(&servo_3, (int16)pwm3);
    set_servo_duty(&servo_4, (int16)pwm4);
}


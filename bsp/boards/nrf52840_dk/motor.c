#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

typedef struct
{
    uint8_t Motor_Num;     // 电机编号，用于识别电机
    uint8_t INA_Port;      // 对应pwm的IO的Port; INA: IN1 or IN3
    uint8_t INA_Pin;       // 对应pwm的IO的Pin
    uint8_t INB_Port;      // 对应pwm的IO的Port; INB: IN2 or IN4
    uint8_t INB_Pin;       // 对应pwm的IO的Pin
    bool    Direction_correct;// 转向纠正
    uint8_t DeadZone;      // 电机死区
    uint16_t Max_Duty;     // 最大占空比
} Motor_Table;

// 电机配置表
//Motor_Table Motor_Config[] = {
//    {1, 0, 2, 0, 28, true, 0, 1000},  // 1号电机： P0.2 P0.28
//    {2, 0, 4, 0, 5,  true, 0, 1000},  // 2号电机: P0.4 P0.5
//    {3, 1, 9, 0, 11, true, 0, 1000},  // 3号电机: P1.9 P0.11
//};

Motor_Table Motor_Config[] = {
    {1, 1, 8, 1, 7, true, 0, 1000},  // 1号电机： P1.8 P1.7
    {2, 1, 6, 1, 5, true, 0, 1000},  // 2号电机: P1.6 P1.5
    {3, 1, 4, 1, 3, true, 0, 1000},  // 3号电机: P1.4 P1.3
};

// 电机数量
#define MOTOR_COUNT (sizeof(Motor_Config) / sizeof(Motor_Config[0]))

/**
 * @brief 控制电机转动（包含死区补偿和转向纠正）
 * @param motor_num 电机序号(1-3)
 * @param speed 速度值(-Max_Duty ~ Max_Duty)，正负表示方向
 * @return 错误码: 0-成功, 1-占空比超限, 2-引脚未配置, 3-电机序号无效
 */
uint8_t motor_speed_set(uint8_t motor_num, int16_t speed) {
    // 查找电机配置
    const Motor_Table* motor = NULL;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (Motor_Config[i].Motor_Num == motor_num) {
            motor = &Motor_Config[i];
            break;
        }
    }
    if (!motor) return 3;  // 电机序号无效

    // 转向纠正（在方向判断前执行）
    if (motor->Direction_correct) {
        speed *= -1;
    }

    // 处理方向和占空比
    uint16_t duty = (speed >= 0) ? speed : -speed;
    
    // 死区补偿和范围限制
    if (duty > 0) {
        // 低于死区时提升到死区
        if (duty < motor->DeadZone) {
            duty = motor->DeadZone;
        }
        // 限制最大占空比（按原设计返回错误码）
        if (duty > motor->Max_Duty) {
            return 1;  // 占空比超限
        }
    }

    // 设置PWM输出（修正方向判断逻辑）
    uint8_t ret;
    if (speed > 0) {
        // 正转：INA输出PWM，INB输出0
        ret = pwm_set(motor->INA_Port, motor->INA_Pin, duty);
        if (ret == 0) {
            pwm_set(motor->INB_Port, motor->INB_Pin, 0);
        }
    } else if (speed < 0) {
        // 反转：INB输出PWM，INA输出0
        ret = pwm_set(motor->INB_Port, motor->INB_Pin, duty);
        if (ret == 0) {
            pwm_set(motor->INA_Port, motor->INA_Pin, 0);
        }
    } else {
        // 停止：双引脚均输出0
        ret = pwm_set(motor->INA_Port, motor->INA_Pin, 0);
        if (ret == 0) {
            pwm_set(motor->INB_Port, motor->INB_Pin, 0);
        }
    }
    
    return ret;
}

// 小车物理参数
#define HALF_TRACK       1.00f   // 半轮距系数（轮子到小车中心的距离）

/**
 * @brief 三轮全向轮小车基础控制函数
 * @details 包含参数补偿、范围限制和运动学计算
 * @param V 期望平移速度百分比（0.00~100.00）
 * @param theta 运动方向（任意角度，自动归一化到0~359.99）
 * @param omega 期望旋转速度百分比（0.00~100.00）
 */
void car_control(float V, float theta, float omega) {
    float V_0 = 0;         // 速度补偿
    float theta0 = 0;      // 角度补偿（-180.00~180.00）
    float omega_0 = -0.2;     // 旋转速度补偿

    // 1. 参数补偿
    V += V_0;
    theta += theta0;
    omega += omega_0;

    // 2. 参数范围限制
    const float V_min = 0, V_max = 100;
    V = (V < V_min) ? V_min : (V > V_max) ? V_max : V;

    theta = fmodf(theta, 360.0f);// 角度归一化（0~359.99度）
    if (theta < 0)theta += 360.0f;
    if (theta >= 360.0f) theta = 0.0f;
    float theta_rad = theta * M_PI / 180.0f;  // 转换为弧度

    const float omega_min = -100, omega_max = 100;
    omega = (omega < omega_min) ? omega_min : (omega > omega_max) ? omega_max : omega;

    // 3. 计算各轮速度分量（三轮全向轮布局）
    const float rad30 = 30 * M_PI / 180.0f;  // 30°弧度值
    float S_Comp[3] = {};  // 平移速度分量
    float R_Comp = omega * HALF_TRACK;       // 旋转分量

    S_Comp[0] = -V * sinf(theta_rad + rad30);
    S_Comp[1] =  V * cosf(theta_rad);
    S_Comp[2] =  V * sinf(theta_rad - rad30);

    // 4. 计算最终速度并转换为占空比
    float speed[3];
    for (uint8_t i = 0; i < 3; i++) {
        speed[i] = S_Comp[i] + R_Comp;
        speed[i] = speed[i] / 100.00f * Motor_Config[i].Max_Duty;
    }

    // 5. 速度归一化（关键步骤）
    float max_speed = 0.0f;
    // 找到最大绝对值速度
    for (uint8_t i = 0; i < 3; i++) {
        float abs_speed = fabsf(speed[i]);
        if (abs_speed > max_speed) {
            max_speed = abs_speed;
        }
    }
    // 若超过最大占空比，按比例缩放
    if (max_speed > Motor_Config[0].Max_Duty) {  // 假设所有电机Max_Duty相同
        float scale = Motor_Config[0].Max_Duty / max_speed;
        for (uint8_t i = 0; i < 3; i++) {
            speed[i] *= scale;
        }
    }

    // 5. 驱动电机
    for (uint8_t i = 0; i < 3; i++) {
        motor_speed_set(Motor_Config[i].Motor_Num, (int16_t)speed[i]);
    }
}

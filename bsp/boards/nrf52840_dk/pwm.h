/**
 * @file pwm.h
 * @brief NRF52840 PWM控制器驱动头文件
 * @details 提供PWM控制器的配置和操作函数
 */

#ifndef __PWM_H
#define __PWM_H

#include "nrf52840.h"
#include <stdint.h>
#include <stdbool.h>


/** PWM实例基地址映射 */
#define PWM_BASE_ADDR(instance) ((uint32_t)( \
    (instance == PWM_0) ? NRF_PWM0_BASE : \
    (instance == PWM_1) ? NRF_PWM1_BASE : \
    (instance == PWM_2) ? NRF_PWM2_BASE : \
    (instance == PWM_3) ? NRF_PWM3_BASE : 0 \
))

/** GPIO基地址 */
#define GPIO_PORT0_BASE NRF_P0_BASE
#define GPIO_PORT1_BASE NRF_P1_BASE

/** 寄存器偏移量（基于芯片手册） */
#define PWM_TASKS_STOP              0x004   // 停止所有通道的PWM，并停止序列播放
#define PWM_TASKS_SEQSTART(n)      (0x008 + (n)*0x4) // 从序列n所有通道的第一个pwm
#define PWM_TASKS_NEXTSTEP          0x010   // 在所有启用的通道上按当前序列逐个值步进

//#define PWM_SUBSCRIBE_STOP          0x084   // 任务STOP的订阅配置
//#define PWM_SUBSCRIBE_SEQSTART(n)  (0x088 + (n)*0x4)// 任务SEQSTART(n)的订阅配置
//#define PWM_SUBSCRIBE_NEXTSTEP      0x090   // NEXTSTEP任务的订阅配置

#define PWM_EVENTS_STOPPED          0x104   // 对停止任务响应，当 PWM 脉冲不再生成时发出
#define PWM_EVENTS_SEQSTARTED(n)   (0x108 + (n)*0x4)   // 第一个PWM周期在序列n上开始
#define PWM_EVENTS_SEQEND(n)       (0x110 + (n)*0x4)   // 在每个序列n的末尾发出，当RAM中的最后一个值已应用于计数器时
#define PWM_EVENTS_PWMPERIODEND     0x118   // 在每个 PWM 周期结束时发出
#define PWM_EVENTS_LOOPSDONE        0x11C   // 串联序列已按照 LOOP.CNT 中定义的次数播放完毕

//#define PWM_PUBLISH_STOPPED         0x184   // 发布事件STOPPED的配置
//#define PWM_PUBLISH_SEQSTARTED(n)  (0x188 + (n)*0x4)   // 发布事件SEQSTARTED(n)的配置
//#define PWM_PUBLISH_SEQEND(n)      (0x190 + (n)*0x4)   // 发布事件SEQEND(n)的配置
//#define PWM_PUBLISH_PWMPERIODEND    0x198   // 发布事件PWMPERIODEND的配置
//#define PWM_PUBLISH_LOOPSDONE       0x19C   // 发布事件LOOPSDONE的配置

#define PWM_SHORTS                  0x200   // 本地事件和任务之间的快捷方式
#define PWM_INTEN                   0x300   // 启用或禁用中断
#define PWM_INTENSET                0x304   // 使能中断
#define PWM_INTENCLR                0x308   // 禁用中断

#define PWM_ENABLE                  0x500   // PWM模块使能寄存器
#define PWM_MODE                    0x504   // 选择波计数器的操作模式
#define PWM_COUNTERTOP              0x508   // 脉冲发生器计数器的计数值上限
#define PWM_PRESCALER               0x50C   // PWM_CLK的配置
#define PWM_DECODER                 0x510   // 解码器配置（分LOAD和MODE字段）
#define PWM_LOOP                    0x514   // 循环播放次数

#define PWM_SEQ_PTR(n)             (0x520 + (n)*0x20)   // 序列n在RAM中的起始地址
#define PWM_SEQ_CNT(n)             (0x524 + (n)*0x20)   // 该序列中的值（占空比）长度数量
#define PWM_SEQ_REFRESH(n)         (0x528 + (n)*0x20)   // 在将样本加载到比较寄存器之间增加的PWM周期数
#define PWM_SEQ_ENDDELAY(n)        (0x52C + (n)*0x20)   // 在序列后添加的时间

#define PWM_PSEL_OUT(n)            (0x560 + (n)*0x4)    // PWM通道n的输出引脚选择


/**
 * @enum pwm_instance
 * @brief PWM实例枚举
 */
typedef enum {
    PWM_0,
    PWM_1,
    PWM_2,
    PWM_3,
    PWM_MAX
} pwm_instance;

/**
 * @enum pwm_channel
 * @brief PWM通道枚举
 */
typedef enum {
    PWM_CHANNEL_0,
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_MAX
} pwm_channel;

/**
 * @enum pwm_mode
 * @brief PWM计数器工作模式
 */
typedef enum {
    PWM_MODE_UP = 0,           // 边缘对齐（向上计数）
    PWM_MODE_UP_AND_DOWN = 1   // 中心对齐（上下计数）
} pwm_mode;

/**
 * @enum pwm_polarity
 * @brief PWM输出极性
 */
typedef enum {
    PWM_POLARITY_RISING = 0,   // 上升沿极性
    PWM_POLARITY_FALLING = 1   // 下降沿极性
} pwm_polarity;

/**
 * @enum pwm_prescaler
 * @brief PWM时钟预分频器配置
 */
typedef enum {
    PWM_PRESCALER_DIV_1    = 0, // 16 MHz
    PWM_PRESCALER_DIV_2    = 1, // 8 MHz
    PWM_PRESCALER_DIV_4    = 2, // 4 MHz
    PWM_PRESCALER_DIV_8    = 3, // 2 MHz
    PWM_PRESCALER_DIV_16   = 4, // 1 MHz
    PWM_PRESCALER_DIV_32   = 5, // 500 kHz
    PWM_PRESCALER_DIV_64   = 6, // 250 kHz
    PWM_PRESCALER_DIV_128  = 7  // 125 kHz
} pwm_prescaler;

/**
 * @enum pwm_decoder_load
 * @brief PWM解码器LOAD模式配置
 */
typedef enum {
    PWM_DECODER_LOAD_COMMON     = 0,  // 所有通道共用占空比
    PWM_DECODER_LOAD_GROUPED    = 1,  // 0-1通道一组；2-3通道一组
    PWM_DECODER_LOAD_INDIVIDUAL = 2,  // 每个通道独立占空比
    PWM_DECODER_LOAD_WAVEFORM   = 3   //波形模式（含COUNTERTOP更新）
} pwm_decoder_load;

/**
 * @enum pwm_decoder_mode
 * @brief PWM解码器更新模式配置
 */
typedef enum {
    PWM_DECODER_MODE_RefreshCount = 0,  // 按REFRESH计数更新
    PWM_DECODER_MODE_NextStep = 1       // 按NEXTSTEP任务更新
} pwm_decoder_mode;

/**
 * @struct pwm_pin_config
 * @brief PWM通道引脚配置
 */
typedef struct {
    uint8_t   pin;        // 引脚编号0-31
    uint8_t   port;       // 端口0-1
    bool      connected;  // 是否连接引脚
} pwm_pin_config;

/**
 * @struct pwm_channel_config
 * @brief PWM通道详细配置
 */
typedef struct {
    pwm_channel     channel;    // 通道号
    pwm_pin_config  pin;        // 引脚配置
    uint16_t            initial_duty;  // 初始占空比（0 ~ COUNTERTOP）
    pwm_polarity    polarity;   // 极性（0: RisingEdge, 1: FallingEdge）
} pwm_channel_config;

/**
 * @struct pwm_config
 * @brief PWM完整配置结构体
 */
typedef struct {
    pwm_instance        instance;      // PWM实例
    pwm_mode            mode;          // 计数模式
    pwm_prescaler       prescaler;     // 预分频器
    uint16_t                countertop;    // 计数器最大值（决定周期）
    pwm_decoder_load    decoder_load;  // 解码器LOAD模式
    pwm_decoder_mode    decoder_mode;  // 解码器MODE模式
    pwm_channel_config  channels[4];   // 最多4个通道
    uint8_t                 channel_count; // 实际使用的通道数
    bool                    use_interrupt; // 是否启用中断
} pwm_config;



/**
 * @enum pwm_irq_type
 * @brief PWM中断类型
 */
typedef enum {
    PWM_INT_STOPPED,
    PWM_INT_SEQSTARTED0,
    PWM_INT_SEQSTARTED1,
    PWM_INT_SEQEND0,
    PWM_INT_SEQEND1,
    PWM_INT_PWMPERIODEND,
    PWM_INT_LOOPSDONE,
    PWM_INT_MAX
} pwm_irq_type;



// API函数声明
uint8_t pwm_init(const pwm_config* config);
void pwm_start(pwm_instance instance);
void pwm_stop(pwm_instance instance);
uint8_t pwm_set_duty(pwm_instance instance, pwm_channel channel, uint16_t duty);
void pwm_trigger_refresh(pwm_instance instance);
void pwm_enable_irq(pwm_instance instance, pwm_irq_type irq);
void pwm_disable_irq(pwm_instance instance, pwm_irq_type irq);
bool pwm_get_irq_status(pwm_instance instance, pwm_irq_type irq);
void pwm_clear_irq_status(pwm_instance instance, pwm_irq_type irq);

uint8_t pwm_multi_init(void);
uint8_t pwm_set(uint8_t port, uint8_t pin, uint16_t duty);


#endif
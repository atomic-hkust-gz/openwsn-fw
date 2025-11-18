/**
 * @file pwm.c
 * @brief NRF5340 PWM控制器驱动实现
 * @details 提供PWM控制器的底层配置和操作函数
 * @details 请先根据自身需求对配置表进行修改
 * @details 接着在主函数中调用nrf_pwm_multi_init(void)；进行初始化
 * @details 最后仅需调用nrf_pwm_set(uint8_t port, uint8_t pin, uint16_t duty);对应即可输出pwm信号
 */
#include "pwm.h"
#include <string.h>
#include <stdlib.h>


//------------------------------------------------------------------------------------------------------
//-----------------------如果需要修改PWM配置只需修改以下两个配置表即可---------------------------------------
//------------------------------------------------------------------------------------------------------

/** PWM实例配置表 */
// 定义命名的实例配置结构体类型
typedef struct {
    pwm_instance instance;            // PWM实例
    pwm_mode mode;                    // 计数模式
    pwm_prescaler prescaler;          // 预分频器
    uint16_t countertop;                  // 计数器最大值（决定周期）
    pwm_decoder_load decoder_load;    // 解码器LOAD模式
    pwm_decoder_mode decoder_mode;    // 解码器MODE模式
    bool use_interrupt;                   // 是否启用中断
} pwm_instance_config_t;                
                                         
/** PWM实例配置表 */
static const pwm_instance_config_t pwm_instance_configs[] = {
    {
        .instance = PWM_0,
        .mode = PWM_MODE_UP,
        .prescaler = PWM_PRESCALER_DIV_1,
        .countertop = 1250,
        .decoder_load = PWM_DECODER_LOAD_INDIVIDUAL,
        .decoder_mode = PWM_DECODER_MODE_RefreshCount,
        .use_interrupt = false
    },
   {
        .instance = PWM_1,
        .mode = PWM_MODE_UP,
        .prescaler = PWM_PRESCALER_DIV_1,
        .countertop = 1250,
        .decoder_load = PWM_DECODER_LOAD_INDIVIDUAL,
        .decoder_mode = PWM_DECODER_MODE_RefreshCount,
        .use_interrupt = false
    }
};
#define PWM_INSTANCE_COUNT (sizeof(pwm_instance_configs) / sizeof(pwm_instance_configs[0]))

typedef struct {
    pwm_instance instance;  // 所属PWM实例
    pwm_channel channel;    // 通道号
    uint8_t port;               // GPIO端口
    uint8_t pin;                // GPIO引脚
    uint16_t initial_duty;      // 初始占空比
    pwm_polarity polarity;  // 极性
} pwm_channel_config_t;

/** 通道配置表 */
static const pwm_channel_config_t pwm_channel_configs[] = {
    {PWM_0, PWM_CHANNEL_0, 1, 8, 0, PWM_POLARITY_RISING},  // P1.8
    {PWM_0, PWM_CHANNEL_1, 1, 7, 0, PWM_POLARITY_RISING},  // P1.7
    {PWM_0, PWM_CHANNEL_2, 1, 6, 0, PWM_POLARITY_RISING},  // P1.6
    {PWM_0, PWM_CHANNEL_3, 1, 5, 0, PWM_POLARITY_RISING},  // P1.5
    {PWM_1, PWM_CHANNEL_0, 1, 4, 0, PWM_POLARITY_RISING},  // P1.4
    {PWM_1, PWM_CHANNEL_1, 1, 3, 0, PWM_POLARITY_RISING},  // P1.3
    {PWM_1, PWM_CHANNEL_2, 1, 2, 0, PWM_POLARITY_RISING},  // P1.2 (not used)
    {PWM_1, PWM_CHANNEL_3, 1, 1, 0, PWM_POLARITY_RISING}   // P1.1 (not used)
};

#define PWM_CHANNEL_CONFIG_COUNT (sizeof(pwm_channel_configs) / sizeof(pwm_channel_configs[0]))

//------------------------------------------------------------------------------------------------------
//-----------------------如果需要修改PWM配置只需修改以上两个配置表即可---------------------------------------
//------------------------------------------------------------------------------------------------------



/** 内部占空比缓冲区(每个实例4个通道) */
static uint16_t pwm_duty_buf[PWM_MAX][PWM_CHANNEL_MAX] = {0};
/** 保存计数器最大值用于范围检查 */
static uint16_t pwm_countertop[PWM_MAX] = {0};

/**
 * @brief 一键初始化所有PWM实例和通道
 * @return 错误码：PWM_SUCCESS-成功；PWM_INIT_PWM0_FAILED-PWM0初始化失败；PWM_INIT_PWM1_FAILED-PWM1初始化失败
 */
uint8_t pwm_multi_init(void) {

    // 1. 初始化所有PWM实例
    for (uint8_t i = 0; i < PWM_INSTANCE_COUNT; i++) {
        // 使用命名结构体类型而不是匿名结构体
        const pwm_instance_config_t* inst_cfg = &pwm_instance_configs[i];
        
        // 配置PWM实例（动态填充countertop）
        pwm_config cfg = {
            .instance = inst_cfg->instance,
            .mode = inst_cfg->mode,
            .prescaler = inst_cfg->prescaler,
            .countertop = inst_cfg->countertop,
            .decoder_load = inst_cfg->decoder_load,
            .decoder_mode = inst_cfg->decoder_mode,
            .use_interrupt = inst_cfg->use_interrupt,
            .channel_count = 0  // 将从通道配置表中填充
        };

        // 2. 从通道配置表中填充当前实例的通道
        uint8_t ch_count = 0;
        for (uint8_t j = 0; j < PWM_CHANNEL_CONFIG_COUNT; j++) {
            const pwm_channel_config_t* ch_cfg = &pwm_channel_configs[j];
            if (ch_cfg->instance == inst_cfg->instance) {
                if (ch_count < PWM_CHANNEL_MAX) {
                    cfg.channels[ch_count] = (pwm_channel_config){
                        .channel = ch_cfg->channel,
                        .pin = {.port = ch_cfg->port, .pin = ch_cfg->pin, .connected = true},
                        .initial_duty = ch_cfg->initial_duty,
                        .polarity = ch_cfg->polarity
                    };
                    ch_count++;
                }
            }
        }
        cfg.channel_count = ch_count;

        // 3. 初始化并启动PWM
        if (pwm_init(&cfg) != 0) {
            return 2 + i;  // 返回错误码（2=PWM_0失败，3=PWM_1失败）
        }
        pwm_start(inst_cfg->instance);
    }
    return 0;
}

/**
 * @brief 按引脚设置PWM占空比
 * @param port GPIO端口（0或1）
 * @param pin 引脚编号（如28、29、0、1）
 * @param duty 占空比值（0 ~ 对应实例的countertop）
 * @return 错误码：PWM_SUCCESS-成功；PWM_DUTY_OUT_OF_RANGE-占空比超限；PWM_PIN_NOT_FOUND-引脚未配置
 */
uint8_t pwm_set(uint8_t port, uint8_t pin, uint16_t duty) {
    for (uint8_t i = 0; i < PWM_CHANNEL_CONFIG_COUNT; i++) {
        const pwm_channel_config_t* ch_cfg = &pwm_channel_configs[i];
        if (ch_cfg->port == port && ch_cfg->pin == pin) {
            // 直接使用已初始化的 pwm_countertop 数组
            if (duty > pwm_countertop[ch_cfg->instance]) {
                return 1;  // 占空比超出范围
            }
            return pwm_set_duty(ch_cfg->instance, ch_cfg->channel, duty);
        }
    }
    return 2;  // 引脚未配置
}

/**
 * @brief 写入PWM寄存器
 * @param instance PWM实例
 * @param offset 寄存器偏移量
 * @param value 写入值
 */
static void pwm_write_reg(pwm_instance instance, uint32_t offset, uint32_t value) {
    uint32_t base = PWM_BASE_ADDR(instance);
    if (base == 0) return;
    *(volatile uint32_t*)(base + offset) = value;
}

/**
 * @brief 读取PWM寄存器
 * @param instance PWM实例
 * @param offset 寄存器偏移量
 * @return 寄存器值
 */
static uint32_t pwm_read_reg(pwm_instance instance, uint32_t offset) {
    uint32_t base = PWM_BASE_ADDR(instance);
    if (base == 0) return 0;
    return *(volatile uint32_t*)(base + offset);
}

/**
 * @brief 配置GPIO为PWM输出
 * @param pin 引脚配置结构体指针
 */
static void pwm_config_gpio(const pwm_pin_config* pin) {
    // 创建一个非const的临时副本
    pwm_pin_config temp_pin = *pin;
    
    uint32_t gpio_base = (temp_pin.port == 0) ? GPIO_PORT0_BASE : GPIO_PORT1_BASE;
    if (gpio_base == 0) return;

    // 设置为输出
    *(volatile uint32_t*)(gpio_base + 0x018) = 1 << temp_pin.pin; // DIRSET
    // 初始低电平
    *(volatile uint32_t*)(gpio_base + 0x00C) = 1 << temp_pin.pin; // OUTCLR
    // 禁用输入缓冲
    *(volatile uint32_t*)(gpio_base + 0x020 + (temp_pin.pin * 4)) &= ~(1 << 1);
}


/**
 * @brief 初始化PWM控制器
 * @param config 配置结构体指针
 * @return 错误码：0-成功，1-无效实例，2-无效通道数，3-计数器超出范围，4-无效通道号
 */
uint8_t pwm_init(const pwm_config* config) {
  if (config == NULL || config->instance >= PWM_MAX) 
  return 1; //无效实例

  if (config->channel_count == 0 || config->channel_count > PWM_CHANNEL_MAX) 
  return 2; //无效通道数

  if (config->countertop < 3 || config->countertop > 32767) 
  return 3; // 计数器超出范围

pwm_instance instance = config->instance;
  pwm_countertop[instance] = config->countertop;

  // 禁用PWM
  pwm_write_reg(instance, PWM_ENABLE, 0);

  // 配置计数器模式
  pwm_write_reg(instance, PWM_MODE, config->mode);

  // 配置预分频器
  pwm_write_reg(instance, PWM_PRESCALER, config->prescaler);

  // 配置计数器上限
  pwm_write_reg(instance, PWM_COUNTERTOP, config->countertop);

  // 配置解码器：LOAD=指定模式，MODE=RefreshCount（0）（文档12-300）
  uint32_t decoder = (config->decoder_load & 0x03) |
                    ((config->decoder_mode & 0x03)<< 8);
  pwm_write_reg(instance, PWM_DECODER, decoder);

  // 配置通道引脚和占空比缓冲区
  for (uint8_t i = 0; i < config->channel_count; i++) {
      const pwm_channel_config* ch = &config->channels[i];

      if (ch->channel >= PWM_CHANNEL_MAX) 
      return 4; // 无效通道号

      // 引脚配置：CONNECT=0（连接），PORT和PIN
      uint32_t pin_config = (ch->pin.pin & 0x1F) |
                          ((ch->pin.port & 0x01)<< 5) |
                          ((ch->pin.connected ? 0 : 1) << 31);
      pwm_write_reg(instance, PWM_PSEL_OUT(ch->channel), pin_config);

      // 配置GPIO为输出（初始低电平）
      if (ch->pin.connected) {
            pwm_config_gpio(&ch->pin);
      }
      // 初始化序列缓冲区（bit15=极性，bit14-0=占空比）
      pwm_duty_buf[instance][ch->channel] = (ch->polarity << 15) |
                                            (ch->initial_duty & 0x7FFF);
  }

  // 配置序列0：指向全局缓冲区，长度=通道数，每个周期更新
  pwm_write_reg(instance, PWM_SEQ_PTR(0),  (uint32_t)pwm_duty_buf[instance]);
  pwm_write_reg(instance, PWM_SEQ_CNT(0), config->channel_count);
  pwm_write_reg(instance, PWM_SEQ_REFRESH(0), 0); // 每个周期更新
  pwm_write_reg(instance, PWM_SEQ_ENDDELAY(0), 0);

  // 禁用循环
  pwm_write_reg(instance, PWM_LOOP, 0);

  // 配置中断
  if (config->use_interrupt) {
      // 使能周期结束中断便于同步
      pwm_write_reg(instance, PWM_INTENSET, 1 << 6); // PWMPERIODEND
  }

  // 启用PWM
  pwm_write_reg(instance, PWM_ENABLE, 1);

  return 0;
}

/**
 * @brief 启动PWM输出
 * @param instance PWM实例
 */
void pwm_start(pwm_instance instance) {
    if (instance >= PWM_MAX) return;
    // 启动序列0
    pwm_write_reg(instance, PWM_TASKS_SEQSTART(0), 1);
}

/**
 * @brief 停止PWM输出
 * @param instance PWM实例
 */
void pwm_stop(pwm_instance instance) {
    if (instance >= PWM_MAX) return;
    // 停止所有通道
    pwm_write_reg(instance, PWM_TASKS_STOP, 1);
}

/**
 * @brief 设置PWM通道占空比
 * @param instance PWM实例
 * @param channel PWM通道
 * @param duty 占空比值（0~countertop）
 * @return 错误码：0-成功，1-无效实例或通道，2-占空比值超出范围，3-未初始化的实例
 */
uint8_t pwm_set_duty(pwm_instance instance, pwm_channel channel, uint16_t duty) {
    if (instance >= PWM_MAX || channel >= PWM_CHANNEL_MAX)
        return 1;

    if (duty > pwm_countertop[instance])
        return 2;// 检查占空比范围

    if (pwm_countertop[instance] == 0)
        return 3; // 未初始化的实例
    
    // 保留极性位（bit15），更新占空比（bit14-0）
    pwm_duty_buf[instance][channel] = (pwm_duty_buf[instance][channel] & 0x8000) |
                                      (duty & 0x7FFF);
    pwm_trigger_refresh(instance);

    return 0;
}


/**
 * @brief 触发序列刷新（确保更新生效）
 * @param instance PWM实例
 */
void pwm_trigger_refresh(pwm_instance instance) {
    if (instance >= PWM_MAX) return;

    // 根据解码器模式选择刷新方式
    uint32_t decoder = pwm_read_reg(instance, PWM_DECODER);
    if ((decoder >> 8) & 0x01) { // NextStep模式
        pwm_write_reg(instance, PWM_TASKS_NEXTSTEP, 1);
    } else { // RefreshCount模式
        // 重启序列0
        pwm_write_reg(instance, PWM_TASKS_SEQSTART(0), 1);
    }
}



/**
 * @brief 使能PWM中断
 * @param instance PWM实例
 * @param irq 中断类型
 */
void pwm_enable_irq(pwm_instance instance, pwm_irq_type irq) {
    if (instance >= PWM_MAX || irq >= PWM_INT_MAX) return;

    uint32_t mask;
    switch (irq) {
        case PWM_INT_STOPPED:      mask = 1 << 1; break;
        case PWM_INT_SEQSTARTED0:  mask = 1 << 2; break;
        case PWM_INT_SEQSTARTED1:  mask = 1 << 3; break;
        case PWM_INT_SEQEND0:      mask = 1 << 4; break;
        case PWM_INT_SEQEND1:      mask = 1 << 5; break;
        case PWM_INT_PWMPERIODEND: mask = 1 << 6; break;
        case PWM_INT_LOOPSDONE:    mask = 1 << 7; break;
        default: return;
    }
    pwm_write_reg(instance, PWM_INTENSET, mask);
}

/**
 * @brief 禁用PWM中断
 * @param instance PWM实例
 * @param irq 中断类型
 */
void pwm_disable_irq(pwm_instance instance, pwm_irq_type irq) {
    if (instance >= PWM_MAX || irq >= PWM_INT_MAX) return;

    uint32_t mask;
    switch (irq) {
        case PWM_INT_STOPPED:      mask = 1 << 1; break;
        case PWM_INT_SEQSTARTED0:  mask = 1 << 2; break;
        case PWM_INT_SEQSTARTED1:  mask = 1 << 3; break;
        case PWM_INT_SEQEND0:      mask = 1 << 4; break;
        case PWM_INT_SEQEND1:      mask = 1 << 5; break;
        case PWM_INT_PWMPERIODEND: mask = 1 << 6; break;
        case PWM_INT_LOOPSDONE:    mask = 1 << 7; break;
        default: return;
    }
    pwm_write_reg(instance, PWM_INTENCLR, mask);
}

/**
 * @brief 获取PWM中断状态
 * @param instance PWM实例
 * @param irq 中断类型
 * @return 中断状态
 */
bool pwm_get_irq_status(pwm_instance instance, pwm_irq_type irq) {
    if (instance >= PWM_MAX || irq >= PWM_INT_MAX) return false;

    uint32_t events_addr;
    switch (irq) {
        case PWM_INT_STOPPED:      events_addr = PWM_EVENTS_STOPPED; break;
        case PWM_INT_SEQSTARTED0:  events_addr = PWM_EVENTS_SEQSTARTED(0); break;
        case PWM_INT_SEQSTARTED1:  events_addr = PWM_EVENTS_SEQSTARTED(1); break;
        case PWM_INT_SEQEND0:      events_addr = PWM_EVENTS_SEQEND(0); break;
        case PWM_INT_SEQEND1:      events_addr = PWM_EVENTS_SEQEND(1); break;
        case PWM_INT_PWMPERIODEND: events_addr = PWM_EVENTS_PWMPERIODEND; break;
        case PWM_INT_LOOPSDONE:    events_addr = PWM_EVENTS_LOOPSDONE; break;
        default: return false;
    }
    return pwm_read_reg(instance, events_addr) != 0;
}


/**
 * @brief 清除PWM中断状态
 * @param instance PWM实例
 * @param irq 中断类型
 */
void pwm_clear_irq_status(pwm_instance instance, pwm_irq_type irq) {
    if (instance >= PWM_MAX || irq >= PWM_INT_MAX) return;

    uint32_t events_addr;
    switch (irq) {
        case PWM_INT_STOPPED:      events_addr = PWM_EVENTS_STOPPED; break;
        case PWM_INT_SEQSTARTED0:  events_addr = PWM_EVENTS_SEQSTARTED(0); break;
        case PWM_INT_SEQSTARTED1:  events_addr = PWM_EVENTS_SEQSTARTED(1); break;
        case PWM_INT_SEQEND0:      events_addr = PWM_EVENTS_SEQEND(0); break;
        case PWM_INT_SEQEND1:      events_addr = PWM_EVENTS_SEQEND(1); break;
        case PWM_INT_PWMPERIODEND: events_addr = PWM_EVENTS_PWMPERIODEND; break;
        case PWM_INT_LOOPSDONE:    events_addr = PWM_EVENTS_LOOPSDONE; break;
        default: return;
    }
    pwm_write_reg(instance, events_addr, 0);
}

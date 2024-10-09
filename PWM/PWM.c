/**
 * LEDC_TIMER_1_BIT, 40000000
 * LEDC_TIMER_2_BIT, 20000000      
 * LEDC_TIMER_3_BIT, 10000000     
 * LEDC_TIMER_4_BIT,  5000000    
 * LEDC_TIMER_5_BIT,  2500000   
 * LEDC_TIMER_6_BIT,  1250000    \
 * LEDC_TIMER_7_BIT,   625000   
 * LEDC_TIMER_8_BIT,   312500  
 * LEDC_TIMER_9_BIT,   156250   
 * LEDC_TIMER_10_BIT,   78125  
 * LEDC_TIMER_11_BIT,   39062  
 * LEDC_TIMER_12_BIT,   19531    
 * LEDC_TIMER_13_BIT,    9765  
 * LEDC_TIMER_14_BIT,    4882 
 * LEDC_TIMER_15_BIT,    2441  
 * LEDC_TIMER_16_BIT,    1220  
 * LEDC_TIMER_17_BIT,     610  
 * LEDC_TIMER_18_BIT,     305
 * LEDC_TIMER_19_BIT,     152 
 * LEDC_TIMER_20_BIT,      76 
 */

#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "PWM.h"

static bool isInit = false;
uint32_t motor_ratios[] = {0, 0, 0, 0};

ledc_channel_config_t motors_channel[4] = {
    {
        .channel = LEDC_CHANNEL_2,//指定 LEDC 模块的通道号。每个通道可以输出一个独立的 PWM 信号。
        .duty = 0,//配置初始的占空比（PWM 占空比），范围通常是 0 到 2^bit_resolution-1。占空比决定了输出 PWM 信号的“高”电平时间长度。在这里，初始占空比被设置为 0，意味着输出信号将从零开始（即电机不转动）。
        .gpio_num = MOT_TOP_RIGHT,//指定将 PWM 信号输出到的 GPIO 引脚。这个引脚连接到对应的电机控制输入端。
        .speed_mode = LEDC_LOW_SPEED_MODE,//选择 PWM 速度模式。ESP32 支持两种速度模式：LEDC_LOW_SPEED_MODE 和 LEDC_HIGH_SPEED_MODE。低速模式通常用于低频率的 PWM 信号。
        .timer_sel = LEDC_TIMER_0//选择 LEDC 模块使用的定时器。LEDC_TIMER_0 表示使用 LEDC 的定时器 0。
    },
    {
        .channel = LEDC_CHANNEL_3,
        .duty = 0,
        .gpio_num = MOT_BOTTOM_RIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    { 
        .channel = LEDC_CHANNEL_4,
        .duty = 0,
        .gpio_num = MOT_BOTTOM_LEFT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = LEDC_CHANNEL_5,
        .duty = 0,
        .gpio_num = MOT_TOP_LEFT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
};

void motors_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        // .clk_cfg          = LEDC_AUTO_CLK  //这一行被注释掉了。如果启用，LEDC_AUTO_CLK 会自动选择时钟源。
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&motors_channel[i]);
    }

    isInit = true;
}

void motors_stop(void)
{
    //motorsDeInit 函数用于停用电机控制系统，具体来说，它停止每个电机的 PWM 信号输出，将电机占空比设置为 0。
    for (int i = 0; i < 4; i++) {
        //motors_channel[i].speed_mode：LED控制器的速度模式（高或低）。motors_channel[i].channel：要停止的通道编号。0：将电机停止时的占空比设置为0%。
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
}

void motors_idle(void)
{
    //motors_idle 函数用于电机怠速，具体来说，它为每个电机的占空比设置为 100。
    for (int i = 0; i < 4; i++) {
        uint16_t ratio;
        ratio = 100;
        ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, ratio);
        ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
    }
}

void motors_PWM(uint32_t id, uint16_t ithrust)
{
    if (isInit) {//检查电机系统是否已初始化。如果未初始化，函数将不会执行进一步的操作。
        uint16_t ratio;
        ratio = ithrust;//将输入的推力值 ithrust 赋值给 ratio 变量。
        //使用 ledc_set_duty 函数设置 PWM 的占空比。这里通过 motorsConv16ToBits 函数将 16 位比例值转换为目标的 PWM 占空比值。
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, ratio);
        //更新电机的 PWM 输出，以便新的占空比生效。
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);

        //将最终的占空比（比例）存储在 motor_ratios 数组中，用于后续的查询或调整。
        motor_ratios[id] = ratio;
    }
}
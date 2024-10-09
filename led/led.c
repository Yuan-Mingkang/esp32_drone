#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "led.h"

void configure_leds(void) {
    gpio_config_t io_conf;
    
    // 配置 GREEN_LED
    io_conf.intr_type = GPIO_INTR_DISABLE;  // 禁止中断
    io_conf.mode = GPIO_MODE_OUTPUT;        // 设置为输出模式
    io_conf.pin_bit_mask = (1ULL << GREEN_LED);  // 选择 GPIO 17
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // 配置 RED_LED
    io_conf.pin_bit_mask = (1ULL << RED_LED);  // 选择 GPIO 16
    gpio_config(&io_conf);

    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(RED_LED, 0);
}
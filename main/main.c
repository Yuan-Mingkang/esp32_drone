#include <stdio.h>
#include <string.h>

#include "esp_err.h"  
// #include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "mahony.h"
// #include "mpu6050.h"
#include "wifi.h"
// #include "PWM.h"
#include "system.h"


void app_main()
{
    wifi_task();
    system_init();
    
    xTaskCreatePinnedToCore(system_run, "system", 4096, NULL, 5, NULL, 1);
    // system_run();
    

    // xTaskCreatePinnedToCore(aaa, "aaa", 2048, &channel, 5, NULL, 1); 
}

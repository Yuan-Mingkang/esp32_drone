#include "mpu6050.h"
#include "mpu6050.c"
#include "wifi.h"
#include "wifi.c"
#include "pid.h"
#include "PWM.h"
#include "led.h"

PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

// extern QueueHandle_t udpDataTx;

MPUPacket anglesRx;
UDPPacket udpRx;

#define Roll_MAX 30.0
#define Roll_MIN -30.0
#define Pitch_MAX 30.0
#define Pitch_MIN -30.0

float roll_output;
float pitch_output;
float yaw_output;

int esc[4];
int start = 0;

float scale_value(int x, float angle_max, float angle_min)
{
    return (((angle_max - angle_min) / 1000) * (x) + angle_min);
}


void system_init(void)
{
    configure_leds();
    mpu6050_task();
    motors_init();
    pidInit(&pidRoll, 0.0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 0.012, PID_ROLL_INTEGRATION_LIMIT, false);
    pidInit(&pidPitch, 0.0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 0.012, PID_PITCH_INTEGRATION_LIMIT, false);
    pidInit(&pidYaw, 0.0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 0.012, PID_YAW_INTEGRATION_LIMIT, false);
    motors_stop();
}


void system_run(void)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(12);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        if (xQueueReceive(udpDataTx, &udpRx, portMAX_DELAY) == pdPASS)
        {
            // gpio_set_level(RED_LED, 1);
            // printf("throttle:%d, yaw:%d, roll:%d, pitch:%d\n", udpRx.throttle_channel, udpRx.yaw_channel, udpRx.roll_channel, udpRx.pitch_channel);
        } else {
            printf("WiFiTask: Failed to receive\n");
            // gpio_set_level(RED_LED, 1);
            // vTaskDelay(1000 / portTICK_PERIOD_MS); 
            // gpio_set_level(RED_LED, 0);
        }

        pidRoll.desired = scale_value(udpRx.roll_channel, Roll_MAX, Roll_MIN);
        pidPitch.desired = scale_value(udpRx.pitch_channel, Pitch_MAX, Pitch_MIN);

        if(udpRx.yaw_channel > 510)
        {
            pidYaw.desired += 0.05;
        }
        else if (udpRx.yaw_channel < 490)
        {
            pidYaw.desired -= 0.05;
        }

        if (pidYaw.desired >= 180.0)
        {
            pidYaw.desired = pidYaw.desired - 360.0;
        }
        else if (pidYaw.desired <= -180.0) 
        {
            pidYaw.desired = pidYaw.desired + 360.0;
        }

        anglesRx = mpu6050_get_value();
        // printf("Roll=%7.2f, Pitch=%7.2f, Yaw=%7.2f\n", anglesRx.Roll, anglesRx.Pitch, anglesRx.Yaw); 

        //电机解锁
        if (udpRx.throttle_channel < 100 && udpRx.yaw_channel > 750)
        {
            start = 1;
            gpio_set_level(RED_LED, 0);
            gpio_set_level(GREEN_LED, 1);
        }
        //启动电机
        if (start == 1 && udpRx.throttle_channel < 100 && udpRx.yaw_channel > 450 && udpRx.yaw_channel < 550)
        {
            start = 2;
            if (abs(anglesRx.Yaw - pidYaw.desired) > 20)
            {
                pidYaw.desired = anglesRx.Yaw;
            }
            pidInit(&pidRoll, pidRoll.desired, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 0.012, PID_ROLL_INTEGRATION_LIMIT, false);
            pidInit(&pidPitch, pidPitch.desired, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 0.012, PID_PITCH_INTEGRATION_LIMIT, false);
            pidInit(&pidYaw, pidYaw.desired, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 0.012, PID_YAW_INTEGRATION_LIMIT, false);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        //停止电机
        if (start == 2 && udpRx.throttle_channel < 100 && udpRx.yaw_channel < 350)
        {
            start = 0;
            gpio_set_level(RED_LED, 1);
            gpio_set_level(GREEN_LED, 0);
        }


        roll_output  =  pidUpdate(&pidRoll, anglesRx.Roll, true);
        pitch_output =  pidUpdate(&pidPitch, anglesRx.Pitch, true);
        yaw_output   =  pidUpdate(&pidYaw, anglesRx.Yaw, true);
        // printf("Roll-out=%7.2f, Pitch-out=%7.2f, Yaw-out=%7.2f\n", roll_output, pitch_output, yaw_output); 

        if(start == 2)
        {
            esc[0] = udpRx.throttle_channel + pitch_output - roll_output + yaw_output;
            esc[1] = udpRx.throttle_channel - pitch_output - roll_output - yaw_output;
            esc[2] = udpRx.throttle_channel - pitch_output + roll_output + yaw_output;
            esc[3] = udpRx.throttle_channel + pitch_output + roll_output - yaw_output;
            for (int i = 0; i < 4; i++) {
                if (esc[i] < 100) esc[i] = 100;
                if (esc[i] > 1000) esc[i] = 1000;
                motors_PWM(i, esc[i]);
            }
        }
        else
        {
            for (int i = 0; i < 4; i++) {
                esc[i] = 0;
                motors_PWM(i, esc[i]);
            }
        }

        printf("esc0=%d, esc1=%d, esc2=%d, esc3=%d\n", esc[0], esc[1], esc[2], esc[3]); 
                
        // printf("Roll=%7.2f, Pitch=%7.2f\n", pidRoll.desired, pidPitch.desired); 

        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
    
}
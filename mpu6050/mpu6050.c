#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include <math.h>
#include "mahony.h"
#include "mpu6050.h"
#include "led.h"

#define MPU6050_I2C_SDA 21
#define MPU6050_I2C_SCL 22
#define MPU6050_I2C_PORT_NUM 0
#define MPU6050_I2C_FREQ 400000
#define MPU6050_ADDR 0x68  //器件地址： b110100(AD0) 

#define WRITE_BIT I2C_MASTER_WRITE  //I2C master write 
#define READ_BIT I2C_MASTER_READ    //I2C master read 
#define ACK_CHECK_EN 0x1            //I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0           //I2C master will not check ack from slave 
#define ACK_VAL 0x0                 //I2C ack value 
#define NACK_VAL 0x1                //I2C nack value    
#define PI		3.14159265358979323846

float accel_sensitivity = 2.0 / 32768.0;
float gyro_sensitivity = 2000.0 / 32768.0;

SensorCalibration calibrationData;


static esp_err_t  i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *reg_addr, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
* @brief 初始化 mpu6050
*/
esp_err_t mpu6050_init()
{
    esp_err_t esp_err;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MPU6050_I2C_SCL,         // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ,  // select frequency specific to your project
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err = i2c_param_config(MPU6050_I2C_PORT_NUM, &conf);
    printf("i2c_param_config: %d \n", esp_err);

    esp_err = i2c_driver_install(MPU6050_I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
    printf("i2c_driver_install: %d \n", esp_err);

    for (size_t i = 0; i < 11; i++)
    {
        esp_err = i2c_master_write_slave(MPU6050_I2C_PORT_NUM, mpu6050_init_cmd[i], 2);
        if (i == 0)
            vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    printf("mpu6050_init_cmd: %d \n", esp_err);
    return esp_err;
}

void calibrate_sensors(SensorCalibration *calib) {
    uint8_t *calibrate_bytes_out = (uint8_t *)malloc(14);
    
    int32_t accel_total[3] = {0};//用于累计加速度计和陀螺仪数据。
    int32_t gyro_total[3] = {0};
    int samples = 500;
    for (int i = 0; i < samples; i++) {
        i2c_master_read_slave(MPU6050_I2C_PORT_NUM, 0x3B, calibrate_bytes_out, 14);
        accel_total[0] += (int16_t)(calibrate_bytes_out[0]<<8 | calibrate_bytes_out[1]);
        accel_total[1] += (int16_t)(calibrate_bytes_out[2]<<8 | calibrate_bytes_out[3]);
        accel_total[2] += (int16_t)(calibrate_bytes_out[4]<<8 | calibrate_bytes_out[5]);

        gyro_total[0] += (int16_t)(calibrate_bytes_out[8]<<8 | calibrate_bytes_out[9]);
        gyro_total[1] += (int16_t)(calibrate_bytes_out[10]<<8 | calibrate_bytes_out[11]);
        gyro_total[2] += (int16_t)(calibrate_bytes_out[12]<<8 | calibrate_bytes_out[13]);

        int countdown = samples - i; // Decrementing countdown
        if (countdown % 10 == 0){          
            printf("\rCalibration samples remaining: %d    ", countdown);
            fflush(stdout); // Ensure the buffer is flushed immediately         
        }
        gpio_set_level(GREEN_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(GREEN_LED, 0);
    }
    int acceleromter_range =  pow(2.0, 1);
    int16_t lsb_per_g = 32768 / acceleromter_range; 

    calib->accel_offset[0] = accel_total[0] / samples;
    calib->accel_offset[1] = accel_total[1] / samples;
    calib->accel_offset[2] = accel_total[2] / samples; 
    calib->accel_offset[2] = accel_total[2] / samples - lsb_per_g;  // Adjust for 1G in z-axis

    calib->gyro_offset[0] = gyro_total[0] / samples;
    calib->gyro_offset[1] = gyro_total[1] / samples;
    calib->gyro_offset[2] = gyro_total[2] / samples;
    // gpio_set_level(GREEN_LED, 0);
}

float safe_asin(float v)
{
	if (isnan(v)) {
		return 0.0f;
	}
	if (v >= 1.0f) {
		return PI/2;
	}
	if (v <= -1.0f) {
		return -PI/2;
	}
	return asin(v);
}

void mpu6050_task(void)
{
    mpu6050_init();
    memset(&calibrationData, 0, sizeof(SensorCalibration));
    calibrate_sensors(&calibrationData);
    // xTaskCreatePinnedToCore(mpu6050_get_value, "MPU6050", 2048, &calibrationData, 5, NULL, 0); 
}

/**
* @brief 读取加速度计、温度和陀螺仪数据
*/
MPUPacket mpu6050_get_value(void)
{
    uint8_t measurement_bytes_out[14];
    // uint64_t start_time, end_time;

    // start_time = esp_timer_get_time();
    i2c_master_read_slave(MPU6050_I2C_PORT_NUM, 0x3B, measurement_bytes_out, 14);
    measurement_out_t measurement_out = {
        .accel_out.accel_xout = ((int16_t)(measurement_bytes_out[0]<<8 | measurement_bytes_out[1]) - calibrationData.accel_offset[0]),
        .accel_out.accel_yout = ((int16_t)(measurement_bytes_out[2]<<8 | measurement_bytes_out[3]) - calibrationData.accel_offset[1]),
        .accel_out.accel_zout = ((int16_t)(measurement_bytes_out[4]<<8 | measurement_bytes_out[5]) - calibrationData.accel_offset[2]),
        .temp_out.temp_xout = (int16_t)(measurement_bytes_out[6]<<8 | measurement_bytes_out[7]),
        .gyro_out.gyro_xout = ((int16_t)(measurement_bytes_out[8]<<8 | measurement_bytes_out[9]) - calibrationData.gyro_offset[0]),
        .gyro_out.gyro_yout = ((int16_t)(measurement_bytes_out[10]<<8 | measurement_bytes_out[11]) - calibrationData.gyro_offset[1]),
        .gyro_out.gyro_zout = ((int16_t)(measurement_bytes_out[12]<<8 | measurement_bytes_out[13]) - calibrationData.gyro_offset[2]),
    };
    float accel_x = measurement_out.accel_out.accel_xout * accel_sensitivity;
    float accel_y = measurement_out.accel_out.accel_yout * accel_sensitivity;
    float accel_z = measurement_out.accel_out.accel_zout * accel_sensitivity;
    //deg/s
    float gyro_x = measurement_out.gyro_out.gyro_xout * gyro_sensitivity;
    float gyro_y = measurement_out.gyro_out.gyro_yout * gyro_sensitivity;
    float gyro_z = measurement_out.gyro_out.gyro_zout * gyro_sensitivity;
    // printf("Accel: X=%7.2f G, Y=%7.2f G, Z=%7.2f G   |   Gyro: X=%7.2f deg/s, Y=%7.2f deg/s, Z=%7.2f\n",
    // accel_x, accel_y, accel_z, 
    // gyro_x, gyro_y, gyro_z);
    //rad/s
    gyro_x = gyro_x * (PI / 180.0f);
    gyro_y = gyro_y * (PI / 180.0f);
    gyro_z = gyro_z * (PI / 180.0f);

    MahonyAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

    // printf("q0=%7.2f, q1=%7.2f, q2=%7.2f, q3=%7.2f\n", q0, q1, q2, q3);
    anglesTx.Pitch = (atan2(2.0f*(q0*q1 + q2*q3), 1 - 2.0f*(q1*q1 + q2*q2)))* 180/PI;	
    anglesTx.Roll = -safe_asin(2.0f*(q0*q2 - q1*q3))* 180/PI;
    anglesTx.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/PI; // yaw

    return anglesTx;
    // float Roll = (atan2(2.0f*(q0*q1 + q2*q3), 1 - 2.0f*(q1*q1 + q2*q2)))* 180/PI;	
    // float Pitch = -safe_asin(2.0f*(q0*q2 - q1*q3))* 180/PI;
    // float Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/PI; // yaw
    // printf("Roll=%7.2f, Pitch=%7.2f, Yaw=%7.2f\n", Roll, Pitch, Yaw); 
       
    // end_time = esp_timer_get_time();
    // uint64_t elapsed_time = end_time - start_time;
    // printf("Loop iteration time: %llu us\n", elapsed_time);

    // vTaskDelay(pdMS_TO_TICKS(10));

}





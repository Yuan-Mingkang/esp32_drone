#ifndef MPU6050_H_
#define MPU6050_H_

#include "esp_err.h"  
#include "driver/i2c.h" 

typedef struct sensor_data_struct
{
    float g_x;
    float g_y;
    float g_z;
    float a_x;
    float a_y;
    float a_z;
}sensor_data_h;

typedef struct accel_out_tag
{
    int16_t accel_xout;
    int16_t accel_yout; 
    int16_t accel_zout; 
}accel_out_t;

typedef struct temp_out_tag
{
    int16_t temp_xout; 
}temp_out_t;

typedef struct gyro_out_tag
{
    int16_t gyro_xout; 
    int16_t gyro_yout; 
    int16_t gyro_zout; 
}gyro_out_t;

typedef struct measurement_out_tag
{
    accel_out_t accel_out;
    temp_out_t temp_out;
    gyro_out_t gyro_out;
}measurement_out_t;

typedef struct {
    int16_t accel_offset[3];
    int16_t gyro_offset[3];
} SensorCalibration;

typedef struct {
    float Roll;
    float Pitch;
    float Yaw;
} MPUPacket;

static QueueHandle_t mpuDataTx;
static MPUPacket anglesTx;
#define ITEM_SIZE_MPU sizeof(MPUPacket)

static const uint8_t mpu6050_init_cmd[11][2] = {
    {0x6B, 0x80}, // PWR_MGMT_1, DEVICE_RESET  
    // need wait 
    {0x6B, 0x00}, // cleat SLEEP
    {0x1B, 0x18}, // Gyroscope Full Scale Range = ± 2000 °/s
    {0x1C, 0x00}, // Accelerometer Full Scale Range = ± 2g 
    {0x38, 0x00}, // Interrupt Enable.disenable 
    {0x6A, 0x00}, // User Control.auxiliary I2C are logically driven by the primary I2C bus
    {0x23, 0x00}, // FIFO Enable.disenable  
    {0x19, 0x09}, // Sample Rate Divider.Sample Rate = 1KHz / (1 + 99)  
    {0x1A, 0x13}, // EXT_SYNC_SET = GYRO_XOUT_L[0]; Bandwidth = 3
    {0x6B, 0x01}, // Power Management 1.PLL with X axis gyroscope reference
    {0x6C, 0x00}  // Power Management 2
};

static esp_err_t  i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *reg_addr, uint8_t *data_rd, size_t size);
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size);
esp_err_t mpu6050_init();
void calibrate_sensors(SensorCalibration *calib);
MPUPacket mpu6050_get_value(void);
void mpu6050_task(void);

#endif
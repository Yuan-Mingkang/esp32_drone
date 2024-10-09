idf_component_register(SRCS "mpu6050.c" 
                    INCLUDE_DIRS "."
                    REQUIRES mahony driver led esp_timer)
idf_component_register(SRCS "system.c" 
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050 wifi PID)
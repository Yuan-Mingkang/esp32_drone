# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(__PROJECT_GROUP_LINK_COMPONENTS ON)

set(PLANE_COMPONENT_DIRS "./wifi"
                         "./mpu6050"
                         "./mahony" 
                         "./PWM"
                         "./PID"
                         "./led"
                         "./system")

set(EXTRA_COMPONENT_DIRS "${EXTRA_COMPONENT_DIRS} ${PLANE_COMPONENT_DIRS}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(01W-idf)
idf_component_register(
    SRCS "main.c""init.hpp""task.hpp"
    PRIV_REQUIRES
    REQUIRES driver mpu6050 ssd1306 bottom ws2812
    INCLUDE_DIRS ""
)
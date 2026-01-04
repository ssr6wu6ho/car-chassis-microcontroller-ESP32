idf_component_register(
    SRCS "main.c""init.hpp""task.hpp"
    PRIV_REQUIRES spi_flash
    REQUIRES driver mpu6050
    INCLUDE_DIRS ""
)
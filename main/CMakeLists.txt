idf_component_register(
    SRCS "main.cpp"
    PRIV_REQUIRES driver mpu6050 ssd1306 bottom ws2812 esp-dsp ec11 uart-foc servo
    REQUIRES 
    INCLUDE_DIRS "."
)
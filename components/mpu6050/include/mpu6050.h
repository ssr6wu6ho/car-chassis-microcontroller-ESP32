/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MPU6050 driver
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"

    // 定义了加速度计和陀螺仪的量程选项
    // 加速度计：±2g、±4g、±8g、±16g
    typedef enum
    {
        ACCE_FS_2G = 0,
        ACCE_FS_4G = 1,
        ACCE_FS_8G = 2,
        ACCE_FS_16G = 3,
    } mpu6050_acce_fs_t;
    // 陀螺仪：±250°/s、±500°/s、±1000°/s、±2000°/s
    typedef enum
    {
        GYRO_FS_250DPS = 0,
        GYRO_FS_500DPS = 1,
        GYRO_FS_1000DPS = 2,
        GYRO_FS_2000DPS = 3,
    } mpu6050_gyro_fs_t;

    // 中断配置定义
    // 定义了中断引脚的工作模式、活动电平、锁存行为和清除方式
    typedef enum
    {
        INTERRUPT_PIN_ACTIVE_HIGH = 0, /*!< The mpu6050 sets its INT pin HIGH on interrupt */
        INTERRUPT_PIN_ACTIVE_LOW = 1   /*!< The mpu6050 sets its INT pin LOW on interrupt */
    } mpu6050_int_pin_active_level_t;

    typedef enum
    {
        INTERRUPT_PIN_PUSH_PULL = 0, /*!< The mpu6050 configures its INT pin as push-pull */
        INTERRUPT_PIN_OPEN_DRAIN = 1 /*!< The mpu6050 configures its INT pin as open drain*/
    } mpu6050_int_pin_mode_t;

    typedef enum
    {
        INTERRUPT_LATCH_50US = 0,         /*!< The mpu6050 produces a 50 microsecond pulse on interrupt */
        INTERRUPT_LATCH_UNTIL_CLEARED = 1 /*!< The mpu6050 latches its INT pin to its active level, until interrupt is cleared */
    } mpu6050_int_latch_t;

    typedef enum
    {
        INTERRUPT_CLEAR_ON_ANY_READ = 0,   /*!< INT_STATUS register bits are cleared on any register read */
        INTERRUPT_CLEAR_ON_STATUS_READ = 1 /*!< INT_STATUS register bits are cleared only by reading INT_STATUS value*/
    } mpu6050_int_clear_t;
    // 中断配置结构体
    // 用于配置MPU6050的中断引脚
    typedef struct
    {
        gpio_num_t interrupt_pin;                     /*!< GPIO connected to mpu6050 INT pin       */
        mpu6050_int_pin_active_level_t active_level;  /*!< Active level of mpu6050 INT pin         */
        mpu6050_int_pin_mode_t pin_mode;              /*!< Push-pull or open drain mode for INT pin*/
        mpu6050_int_latch_t interrupt_latch;          /*!< The interrupt pulse behavior of INT pin */
        mpu6050_int_clear_t interrupt_clear_behavior; /*!< Interrupt status clear behavior         */
    } mpu6050_int_config_t;
    // 中断位定义，定义了MPU6050支持的各种中断位
    extern const uint8_t MPU6050_DATA_RDY_INT_BIT;      /*!< DATA READY interrupt bit               */
    extern const uint8_t MPU6050_I2C_MASTER_INT_BIT;    /*!< I2C MASTER interrupt bit               */
    extern const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT; /*!< FIFO Overflow interrupt bit            */
    extern const uint8_t MPU6050_MOT_DETECT_INT_BIT;    /*!< MOTION DETECTION interrupt bit         */
    extern const uint8_t MPU6050_ALL_INTERRUPTS;        /*!< All interrupts supported by mpu6050    */

    typedef struct
    {
        int16_t raw_acce_x;
        int16_t raw_acce_y;
        int16_t raw_acce_z;
    } mpu6050_raw_acce_value_t;

    typedef struct
    {
        int16_t raw_gyro_x;
        int16_t raw_gyro_y;
        int16_t raw_gyro_z;
    } mpu6050_raw_gyro_value_t;

    typedef struct
    {
        float acce_x;
        float acce_y;
        float acce_z;
    } mpu6050_acce_value_t;

    typedef struct
    {
        float gyro_x;
        float gyro_y;
        float gyro_z;
    } mpu6050_gyro_value_t;

    typedef struct
    {
        float temp;
    } mpu6050_temp_value_t;

    typedef struct
    {
        float roll;
        float pitch;
    } complimentary_angle_t;

    typedef void *mpu6050_handle_t;

    typedef gpio_isr_t mpu6050_isr_t;

    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    // 创建和删除传感器对象
    /**
     * @brief 创建并初始化MPU6050传感器对象，返回传感器句柄。
     * @param port I2C端口号
     * @param dev_addr 设备地址（通常为0x68或0x69）
     * @return
     *     - NULL Fail
     *     - Others Success
     */
    mpu6050_handle_t mpu6050_create(i2c_master_bus_handle_t bus_handle,
                                    uint16_t dev_addr);
    /**
     * @brief 删除并释放传感器对象，释放相关资源。
     * @param sensor object handle of mpu6050
     */
    void mpu6050_delete(mpu6050_handle_t sensor);

    // 设备识别与电源管理
    /**
     * @brief 获取MPU6050设备ID（WHO_AM_I寄存器值，应为0x68）。用于验证传感器是否连接正常。
     * @param sensor object handle of mpu6050
     * @param deviceid a pointer of device ID
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid);
    /**
     * @brief Wake up MPU6050
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor);

    /**
     * @brief Enter sleep mode
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_sleep(mpu6050_handle_t sensor);
    // 传感器配置
    /**
     * @brief 配置加速度计和陀螺仪的量程（如±2g、±4g、±8g、±16g和±250°/s、±500°/s等）
     * @param sensor object handle of mpu6050
     * @param acce_fs accelerometer full scale range
     * @param gyro_fs gyroscope full scale range
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs);
    /**
     * @brief 获取加速度计的灵敏度（用于将原始数据转换为物理单位）。
     * @param sensor object handle of mpu6050
     * @param acce_sensitivity accelerometer sensitivity
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity);
    /**
     * @brief 获取陀螺仪的灵敏度
     * @param sensor object handle of mpu6050
     * @param gyro_sensitivity gyroscope sensitivity
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity);

    //
    //
    //
    //
    //
    //
    //
    //
    //

    // 中断配置与处理
    /**
     * @brief 配置中断引脚行为（活动电平、推挽/开漏模式、锁存行为等）。
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_configuration mpu6050 INT pin configuration parameters
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or incorrect
     *      - ESP_FAIL Failed to configure INT pin on mpu6050
     */
    esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t *const interrupt_configuration);
    /**
     * @brief 注册中断服务例程（ISR）处理MPU6050中断。.
     *
     * @param sensor object handle of mpu6050
     * @param isr function to handle interrupts produced by mpu6050
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to register ISR
     */
    esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr);
    /**
     * @brief 启用特定中断（如数据就绪、FIFO溢出等）
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_sources bit mask with interrupt sources to enable
     *
     * This function does not disable interrupts not set in interrupt_sources. To disable
     * specific mpu6050 interrupts, use mpu6050_disable_interrupts().
     *
     * To enable all mpu6050 interrupts, pass MPU6050_ALL_INTERRUPTS as the argument
     * for interrupt_sources.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
     */
    esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);
    /**
     * @brief 禁用特定中断。
     *
     * @param sensor object handle of mpu6050
     * @param interrupt_sources bit mask with interrupt sources to disable
     *
     * This function does not enable interrupts not set in interrupt_sources. To enable
     * specific mpu6050 interrupts, use mpu6050_enable_interrupts().
     *
     * To disable all mpu6050 interrupts, pass MPU6050_ALL_INTERRUPTS as the
     * argument for interrupt_sources.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
     */
    esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources);
    /**
     * @brief 获取中断状态
     *
     * @param sensor object handle of mpu6050
     * @param out_intr_status[out] bit mask that is assigned a value representing the interrupts triggered by the mpu6050
     *
     * This function can be used by the mpu6050 ISR to determine the source of
     * the mpu6050 interrupt that it is handling.
     *
     * After this function returns, the bits set in out_intr_status are
     * the sources of the latest interrupt triggered by the mpu6050. For example,
     * if MPU6050_DATA_RDY_INT_BIT is set in out_intr_status, the last interrupt
     * from the mpu6050 was a DATA READY interrupt.
     *
     * The behavior of the INT_STATUS register of the mpu6050 may change depending on
     * the value of mpu6050_int_clear_t used on interrupt configuration.
     *
     * @return
     *      - ESP_OK Success
     *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
     *      - ESP_FAIL Failed to retrieve interrupt status from mpu6050
     */
    esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t sensor, uint8_t *const out_intr_status);
    /**
     * @brief 判断中断是否由数据就绪产生。
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt was not produced due to data ready
     *      - Any other positive integer: Interrupt was a DATA_READY interrupt
     */
    extern uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status);
    /**
     * @brief 判断中断是否为I2C主模式中断。
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt is not an I2C master interrupt
     *      - Any other positive integer: Interrupt was an I2C master interrupt
     */
    extern uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status);
    /**
     * @brief 判断中断是否由FIFO溢出触发。
     *
     * @param interrupt_status mpu6050 interrupt status, obtained by invoking mpu6050_get_interrupt_status()
     *
     * @return
     *      - 0: The interrupt is not a fifo overflow interrupt
     *      - Any other positive integer: Interrupt was triggered by a fifo overflow
     */
    extern uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status);

    //
    //
    //
    //
    //
    //
    //

    // 数据读取
    /**
     * @brief Read raw accelerometer measurements
     *
     * @param sensor object handle of mpu6050
     * @param raw_acce_value raw accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value);

    /**
     * @brief Read raw gyroscope measurements
     *
     * @param sensor object handle of mpu6050
     * @param raw_gyro_value raw gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value);

    /**
     * @brief Read accelerometer measurements
     *
     * @param sensor object handle of mpu6050
     * @param acce_value accelerometer measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value);

    /**
     * @brief Read gyro values
     *
     * @param sensor object handle of mpu6050
     * @param gyro_value gyroscope measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value);

    /**
     * @brief Read temperature values
     *
     * @param sensor object handle of mpu6050
     * @param temp_value temperature measurements
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value);

    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    // 姿态解算
    /**
     * @brief Use complimentory filter to calculate roll and pitch
     *
     * @param sensor object handle of mpu6050
     * @param acce_value accelerometer measurements
     * @param gyro_value gyroscope measurements
     * @param complimentary_angle complimentary angle
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Fail
     */
    esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                           const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle);

#ifdef __cplusplus
}
#endif

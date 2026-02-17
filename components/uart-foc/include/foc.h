#ifndef FOC_H
#define FOC_H

#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // foc.h
    typedef struct
    {
        uart_port_t uart_port;
        char motor_id; // 新增：电机 ID
    } motor_handle_t;

    // 初始化时传入电机 ID
    motor_handle_t *motor_init(uart_port_t port, char motor_id);

    /**
     * @brief 发送命令（阻塞方式）
     * @param handle 电机句柄
     * @param motor_id 电机 ID（如 'A'）
     * @param cmd 命令字符串（不含 ID 和换行）
     * @return ESP_OK 成功，否则失败
     */
esp_err_t motor_send_cmd(motor_handle_t *handle, const char *cmd);

    /**
     * @brief 发送命令并等待响应
     * @param handle 电机句柄
     * @param motor_id 电机 ID
     * @param cmd 命令字符串
     * @param response 接收缓冲区
     * @param resp_len 缓冲区大小
     * @param timeout_ms 超时时间（毫秒）
     * @return ESP_OK 成功，否则失败
     */
   esp_err_t motor_send_and_wait(motor_handle_t *handle, const char *cmd,
                              char *response, size_t resp_len, uint32_t timeout_ms);
    /**
     * @brief 接收响应数据
     * @param handle 电机句柄
     * @param buffer 接收缓冲区
     * @param len 缓冲区大小
     * @param timeout_ms 超时时间
     * @return 接收到的字节数，失败返回 -1
     */
    int motor_receive_response(motor_handle_t *handle, char *buffer, size_t len, uint32_t timeout_ms);

    /**
     * @brief 解析角度响应字符串
     * @param response 原始响应字符串（可能包含多余字符）
     * @param angle 输出角度值
     * @return ESP_OK 成功，否则失败
     */
    esp_err_t motor_parse_angle_response(const char *response, float *angle);

#ifdef __cplusplus
}
#endif

#endif // FOC_H
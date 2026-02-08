#ifndef MOTOR_UART_INIT_H
#define MOTOR_UART_INIT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


// 函数声明
void foc_set_uart_port(uart_port_t port);
esp_err_t motor_uart_init(void);
esp_err_t motor_send_cmd(char motor_id, const char *cmd);
int motor_receive_response(char *buffer, size_t len, uint32_t timeout_ms);
esp_err_t motor_send_and_wait(char motor_id, const char *cmd, char *response, size_t resp_len, uint32_t timeout_ms);

esp_err_t motor_parse_angle_response(const char *response, float *angle);  // 添加这一行
#ifdef __cplusplus
}
#endif
#endif // __MOTOR_UART_INIT_H__
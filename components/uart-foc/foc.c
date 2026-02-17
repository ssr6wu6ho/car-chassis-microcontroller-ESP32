#include "foc.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "foc";

motor_handle_t *motor_init(uart_port_t port, char motor_id)
{
    motor_handle_t *handle = malloc(sizeof(motor_handle_t));
    if (handle)
    {
        handle->uart_port = port;
        handle->motor_id = motor_id;
        ESP_LOGD("foc", "Motor handle created for UART%d, ID=%c", port, motor_id);
    }
    else
    {
        ESP_LOGE("foc", "Failed to allocate motor handle");
    }
    return handle;
}

// 发送命令（阻塞方式）
esp_err_t motor_send_cmd(motor_handle_t *handle, const char *cmd)
{
    if (!handle || !cmd)
    {
        ESP_LOGE(TAG, "Handle or command is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 构建完整命令：ID + 命令 + \r\n
    char full_cmd[64];
    int len = snprintf(full_cmd, sizeof(full_cmd), "%c%s\r\n", handle->motor_id, cmd);
    // 清空接收缓冲区
    uart_flush_input(handle->uart_port);

    // 发送命令
    int sent = uart_write_bytes(handle->uart_port, full_cmd, len);
    if (sent != len)
    {
        ESP_LOGE(TAG, "Write failed: sent %d/%d bytes", sent, len);
        return ESP_FAIL;
    }

    // 等待发送完成
    uart_wait_tx_done(handle->uart_port, pdMS_TO_TICKS(100));

    ESP_LOGD(TAG, "TX: '%s' (len=%d)", full_cmd, len);
    return ESP_OK;
}

// 发送命令并等待响应
esp_err_t motor_send_and_wait(motor_handle_t *handle, const char *cmd,
                              char *response, size_t resp_len, uint32_t timeout_ms)
{
    esp_err_t ret = motor_send_cmd(handle, cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // 等待一小段时间让设备响应
    vTaskDelay(pdMS_TO_TICKS(50));

    // 接收响应
    int received = motor_receive_response(handle, response, resp_len, timeout_ms);
    if (received <= 0)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

// 接收响应

int motor_receive_response(motor_handle_t *handle, char *buffer, size_t len, uint32_t timeout_ms)
{
    if (!handle || !buffer || len == 0)
    {
        ESP_LOGE(TAG, "Invalid parameters");
        return -1;
    }

    int total_received = 0;
    uint32_t start_time = xTaskGetTickCount();

    // 设置接收超时
    uart_set_rx_timeout(handle->uart_port, 10); // 字符间超时

    while (total_received < (len - 1))
    {
        int available = 0;
        uart_get_buffered_data_len(handle->uart_port, (size_t *)&available);

        if (available > 0)
        {
            int to_read = (available < (len - 1 - total_received)) ? available : (len - 1 - total_received);
            int received = uart_read_bytes(handle->uart_port,
                                           (uint8_t *)buffer + total_received,
                                           to_read,
                                           pdMS_TO_TICKS(50));
            if (received > 0)
            {
                total_received += received;

                // 检查是否收到完整响应（以\r\n结尾）
                if (total_received >= 2 &&
                    buffer[total_received - 2] == '\r' &&
                    buffer[total_received - 1] == '\n')
                {
                    break;
                }
            }
        }

        // 检查超时
        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS > timeout_ms)
        {
            if (total_received > 0)
            {
                ESP_LOGW(TAG, "Partial response received (timeout)");
                break;
            }
            else
            {
                ESP_LOGW(TAG, "Receive timeout, no data");
                return -1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 确保字符串以 null 结尾
    buffer[total_received] = '\0';

    // 去掉末尾的 \r\n
    if (total_received >= 2 &&
        buffer[total_received - 2] == '\r' &&
        buffer[total_received - 1] == '\n')
    {
        buffer[total_received - 2] = '\0';
        total_received -= 2;
    }

    if (total_received > 0)
    {
        ESP_LOGD(TAG, "RX raw: '%s' (len=%d)", buffer, total_received);
    }

    return total_received;
}

// 解析角度响应
esp_err_t motor_parse_angle_response(const char *response, float *angle)
{
    if (!response || !angle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 清理响应字符串（只保留数字、小数点、正负号）
    char clean_str[32];
    int j = 0;

    for (int i = 0; response[i] && j < sizeof(clean_str) - 1; i++)
    {
        if (isdigit((unsigned char)response[i]) ||
            response[i] == '.' ||
            response[i] == '-' ||
            response[i] == '+')
        {
            clean_str[j++] = response[i];
        }
        else if (response[i] == ',' || response[i] == ';')
        {
            // 遇到分隔符停止解析
            break;
        }
    }
    clean_str[j] = '\0';

    if (j == 0)
    {
        return ESP_FAIL;
    }

    // 解析浮点数
    char *endptr;
    float val = strtof(clean_str, &endptr);
    if (endptr == clean_str)
    {
        return ESP_FAIL;
    }

    *angle = val;
    return ESP_OK;
}
#include "foc.h"
#include <string.h>
#include <ctype.h>

static uart_port_t s_uart_port = UART_NUM_0; // 默认值0会被覆盖
void foc_set_uart_port(uart_port_t port)
{
    s_uart_port = port;
}

// 发送命令（阻塞方式）
esp_err_t motor_send_cmd(char motor_id, const char *cmd)
{
    if (!cmd)
    {
        ESP_LOGE("motor_uart", "Command is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 构建完整命令：ID + 命令 + \r\n
    char full_cmd[64];
    int len = snprintf(full_cmd, sizeof(full_cmd), "%c%s\r\n", motor_id, cmd);

    // 清空接收缓冲区
    uart_flush_input(s_uart_port);

    // 发送命令
    int sent = uart_write_bytes(s_uart_port, full_cmd, len);
    if (sent != len)
    {
        ESP_LOGE("motor_uart", "Write failed: sent %d/%d bytes", sent, len);
        return ESP_FAIL;
    }

    // 等待发送完成
    uart_wait_tx_done(s_uart_port, pdMS_TO_TICKS(100));

    ESP_LOGD("motor_uart", "TX: '%s' (len=%d)", full_cmd, len);
    return ESP_OK;
}

// 发送命令并等待响应
esp_err_t motor_send_and_wait(char motor_id, const char *cmd,
                              char *response, size_t resp_len,
                              uint32_t timeout_ms)
{
    esp_err_t ret = motor_send_cmd(motor_id, cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // 等待一小段时间让设备响应
    vTaskDelay(pdMS_TO_TICKS(50));

    // 接收响应
    int received = motor_receive_response(response, resp_len, timeout_ms);
    if (received <= 0)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

// 改进的接收函数
int motor_receive_response(char *buffer, size_t len, uint32_t timeout_ms)
{
    if (!buffer || len == 0)
    {
        ESP_LOGE("motor_uart", "Invalid buffer parameters");
        return -1;
    }

    int total_received = 0;
    uint32_t start_time = xTaskGetTickCount();

    // 设置接收超时
    uart_set_rx_timeout(s_uart_port, 10); // 字符间超时

    while (total_received < (len - 1))
    {
        int available = 0;
        uart_get_buffered_data_len(s_uart_port, (size_t *)&available);

        if (available > 0)
        {
            // 读取可用数据
            int to_read = (available < (len - 1 - total_received)) ? available : (len - 1 - total_received);

            int received = uart_read_bytes(s_uart_port,
                                           (uint8_t *)buffer + total_received,
                                           to_read,
                                           pdMS_TO_TICKS(50));

            if (received > 0)
            {
                total_received += received;

                // 检查是否收到了完整响应（以\r\n结尾）
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
                ESP_LOGW("motor_uart", "Partial response received (timeout)");
                break;
            }
            else
            {
                ESP_LOGW("motor_uart", "Receive timeout, no data");
                return -1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 确保字符串以null结尾
    buffer[total_received] = '\0';

    // 去掉末尾的\r\n
    if (total_received >= 2 &&
        buffer[total_received - 2] == '\r' &&
        buffer[total_received - 1] == '\n')
    {
        buffer[total_received - 2] = '\0';
        total_received -= 2;
    }

    if (total_received > 0)
    {
        ESP_LOGD("motor_uart", "RX raw: '%s' (len=%d)", buffer, total_received);
    }

    return total_received;
}

// 辅助函数：解析角度响应
esp_err_t motor_parse_angle_response(const char *response, float *angle)
{
    if (!response || !angle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 清理响应字符串
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
            // 分隔符，可以停止解析
            break;
        }
    }
    clean_str[j] = '\0';

    if (j == 0)
    {
        return ESP_FAIL;
    }

    // 尝试解析为浮点数
    char *endptr;
    float val = strtof(clean_str, &endptr);

    if (endptr == clean_str)
    {
        // 没有成功转换任何字符
        return ESP_FAIL;
    }

    *angle = val;
    return ESP_OK;
}
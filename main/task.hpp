#include "init.hpp"
#include <math.h>

void task_mpu6050GetParam_EKF(void *pvParameter)
{
    const TickType_t OUTPUT_PERIOD = pdMS_TO_TICKS(1000);
    const TickType_t EKF_UPDATE_PERIOD = pdMS_TO_TICKS(20); // 50 Hz EKF
    while (1)
    {
        // 读取原始传感器数据
        mpu6050_get_acce(mpu6050, &mpu6050_acce);
        mpu6050_get_gyro(mpu6050, &mpu6050_gyro);
        mpu6050_get_temp(mpu6050, &mpu6050_temp);

        // 转换数据单位（假设原始数据是g和°/s）
        float accel[3], gyro[3];

        // MPU6050加速度计数据 (转换为m/s²)
        accel[0] = mpu6050_acce.acce_x * 9.80665f; // x轴
        accel[1] = mpu6050_acce.acce_y * 9.80665f; // y轴
        accel[2] = mpu6050_acce.acce_z * 9.80665f; // z轴

        // MPU6050陀螺仪数据 (转换为rad/s)
        const float deg2rad = M_PI / 180.0f;
        gyro[0] = mpu6050_gyro.gyro_x * deg2rad; // x轴
        gyro[1] = mpu6050_gyro.gyro_y * deg2rad; // y轴
        gyro[2] = mpu6050_gyro.gyro_z * deg2rad; // z轴

        // 使用卡尔曼滤波计算姿态
        EKF_MPU6050::Attitude attitude = ekf_filter.update(accel, gyro);

        // 输出结果
        mpu6050_angle.roll = attitude.roll;
        mpu6050_angle.pitch = attitude.pitch;

        // 可选：获取陀螺仪偏差（用于调试）
        float gyro_bias[3];
        ekf_filter.getGyroBias(gyro_bias);
        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms采样周期
    }
}
void task_oled_display_fancy_ui_enhanced(void *pvParameter)
{
// 历史位置记录（用于绘制轨迹效果）
#define HISTORY_SIZE 5
    static int history_x[HISTORY_SIZE] = {0};
    static int history_y[HISTORY_SIZE] = {0};
    static int history_index = 0;

    // 右侧水平仪区域参数
    int center_x = 100;    // 圆心x坐标
    int center_y = 40;     // 圆心y坐标
    int outer_radius = 22; // 外圆半径
    int inner_radius = 18; // 内圆半径（网格）
    int dot_radius = 3;    // 中心点半径

    char roll_str[16];
    char pitch_str[16];
    char temp_str[16];

    while (1)
    {
        // 1. 清除整个屏幕
        ssd1306_clear(oled);

        // 2. 绘制静态元素（每次循环都重新绘制）
        // 绘制标题和装饰线
        ssd1306_draw_text(oled, 2, 4, "MPU6050", true);
        ssd1306_draw_rect(oled, 0, 0, 127, 15, false);

        // 绘制区域分隔线
        ssd1306_draw_line(oled, 68, 15, 68, 63, true); // 垂直线分隔左右

        // 左侧数值区域装饰
        ssd1306_draw_rect(oled, 2, 17, 65, 50, false);
        ssd1306_draw_text(oled, 5, 20, "Angle Data", true);
        ssd1306_draw_line(oled, 5, 30, 55, 30, true); // 标题下划线

        // 绘制右侧静态水平仪元素
        // 绘制外圆
        ssd1306_draw_circle(oled, center_x, center_y, outer_radius, false);

        // 绘制内圆网格
        ssd1306_draw_circle(oled, center_x, center_y, inner_radius, false);

        // 绘制网格线
        for (int i = 0; i < 4; i++)
        {
            float angle = i * M_PI / 2.0f;
            int x1 = center_x + (int)(inner_radius * cos(angle));
            int y1 = center_y + (int)(inner_radius * sin(angle));
            ssd1306_draw_line(oled, center_x, center_y, x1, y1, true);
        }

        // 绘制中心参考点
        ssd1306_draw_circle(oled, center_x, center_y, dot_radius, true);

        // 3. 获取MPU6050数据并绘制动态元素
        // mpu6050_complimentory_filter(mpu6050, &mpu6050_acce, &mpu6050_gyro, &mpu6050_angle);

        // 显示左侧数值
        snprintf(roll_str, sizeof(roll_str), "Roll: %.1f", mpu6050_angle.roll);
        snprintf(pitch_str, sizeof(pitch_str), "Pitch: %.1f", mpu6050_angle.pitch);
        snprintf(temp_str, sizeof(temp_str), "Temp: %.1f", mpu6050_temp.temp / 340.0 + 36.53);

        // 显示新数据
        ssd1306_draw_text(oled, 5, 33, roll_str, true);
        ssd1306_draw_text(oled, 5, 43, pitch_str, true);
        ssd1306_draw_text(oled, 5, 53, temp_str, true);

        // 计算水平仪小球位置
        // 限制角度范围在±30度内
        float limited_roll = mpu6050_angle.roll;
        float limited_pitch = mpu6050_angle.pitch;
        if (limited_roll > 30.0f)
            limited_roll = 30.0f;
        if (limited_roll < -30.0f)
            limited_roll = -30.0f;
        if (limited_pitch > 30.0f)
            limited_pitch = 30.0f;
        if (limited_pitch < -30.0f)
            limited_pitch = -30.0f;

        // 映射到屏幕坐标（注意Y轴方向取反）
        int ball_x = center_x + (int)(limited_roll * inner_radius / 30.0f);
        int ball_y = center_y - (int)(limited_pitch * inner_radius / 30.0f); // Y轴取反

        // 限制在圆圈范围内
        int dx = ball_x - center_x;
        int dy = ball_y - center_y;
        float distance = sqrt(dx * dx + dy * dy);
        if (distance > (inner_radius - dot_radius))
        {
            float scale = (inner_radius - dot_radius) / distance;
            ball_x = center_x + (int)(dx * scale);
            ball_y = center_y + (int)(dy * scale);
        }

        // 保存到历史记录（用于轨迹效果）
        history_x[history_index] = ball_x;
        history_y[history_index] = ball_y;
        history_index = (history_index + 1) % HISTORY_SIZE;

        // 绘制历史轨迹（淡出效果）
        for (int i = 0; i < HISTORY_SIZE; i++)
        {
            int idx = (history_index + i) % HISTORY_SIZE;
            if (history_x[idx] != 0 && history_y[idx] != 0)
            {
                // 较新的轨迹用较大的点，较旧的用较小的点
                int trail_size = 2 - (i / 2); // 递减大小
                if (trail_size > 0)
                {
                    ssd1306_draw_circle(oled, history_x[idx], history_y[idx], trail_size, true);
                }
            }
        }

        // 绘制当前小球
        ssd1306_draw_circle(oled, ball_x, ball_y, dot_radius + 1, false); // 外圈
        ssd1306_draw_circle(oled, ball_x, ball_y, dot_radius, true);      // 填充内圈

        // 4. 刷新显示
        ssd1306_display(oled);

        // 5. 控制刷新率
        vTaskDelay(pdMS_TO_TICKS(20)); // 20Hz刷新率（更平滑）
    }
}
void bottom_driver_task(void *arg)
{

    // 获取按钮句柄
    bottom_handle_t left_btn = left_bottom;
    bottom_handle_t right_btn = right_bottom;

    if (left_btn == NULL || right_btn == NULL)
    {
        ESP_LOGE("BUTTON", "Failed to get button handles");
        return;
    }

    while (1)
    {
        // 更新按钮状态
        button_update(left_btn);
        button_update(right_btn);

        // 检查左按钮事件
        button_event_t left_event = button_get_event(left_btn);
        if (left_event != BUTTON_EVENT_NONE)
        {
            switch (left_event)
            {
            case BUTTON_EVENT_SHORT_PRESS:
                ESP_LOGI("BUTTON", "Left button short press");
                // 处理左按钮短按
                break;
            case BUTTON_EVENT_LONG_PRESS:
                ESP_LOGI("BUTTON", "Left button long press");
                // 处理左按钮长按
                break;
            default:
                break;
            }
        }

        // 检查右按钮事件
        button_event_t right_event = button_get_event(right_btn);
        if (right_event != BUTTON_EVENT_NONE)
        {
            switch (right_event)
            {
            case BUTTON_EVENT_SHORT_PRESS:
                ESP_LOGI("BUTTON", "Right button short press");
                // 处理右按钮短按
                break;
            case BUTTON_EVENT_LONG_PRESS:
                ESP_LOGI("BUTTON", "Right button long press");
                // 处理右按钮长按
                break;
            default:
                break;
            }
        }

        // 检查按钮持续按下状态
        if (button_is_pressed(left_btn))
        {
            // 左按钮持续按下中
        }

        if (button_is_pressed(right_btn))
        {
            // 右按钮持续按下中
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void task_ssd1306_animator(void *pvParameters)
{
    int frame = 0;
    while (1)
    {
        ssd1306_draw_bitmap(oled, 0, 0, frames_eye[frame], FRAME_WIDTH, FRAME_HEIGHT);
        frame = (frame + 1) % FRAME_COUNT_48;
        vTaskDelay(7);
        ssd1306_display(oled);
    }
}
void RGB_task(void *arg)
{
    // 清空
    ws2812_clear();
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        ws2812_rainbow_breathing(3000); // 周期3秒
    }
}
void ec11_test_task(void *arg)
{
    ec11_event_t event;
    int counter = 0;
    while (1)
    {
        // 等待事件（阻塞，最多等待100ms）
        if (ec11_wait_event(ec11_handle, &event, pdMS_TO_TICKS(100)))
        {
            switch (event)
            {
            case EC11_EVENT_CW:
                counter++;
                ESP_LOGI("EC11", "Rotary CW, counter: %d", counter);
                break;

            case EC11_EVENT_CCW:
                counter--;
                ESP_LOGI("EC11", "Rotary CCW, counter: %d", counter);
                break;

            case EC11_EVENT_SHORT_PRESS:
                ESP_LOGI("EC11", "Button short press");
                // 重置计数器
                counter = 0;
                ESP_LOGI("EC11", "Counter reset to 0");
                break;

            case EC11_EVENT_LONG_PRESS:
                ESP_LOGI("EC11", "Button long press");
                // 长按可以做其他操作，比如进入配置模式
                break;

            case EC11_EVENT_RELEASED:
                ESP_LOGI("EC11", "Button released");
                break;

            case EC11_EVENT_NONE:
            default:
                break;
            }
        }
    }
}

static uint16_t calibration_value_0 = 30;    // Real 0 degree angle
static uint16_t calibration_value_180 = 195; // Real 0 degree angle
static void servo_test_task(void *arg)
{
    while (1) {
        // Set the angle of the servo
        for (int i = calibration_value_0; i <= calibration_value_180; i += 1) {
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, i);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        // Return to the initial position
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, calibration_value_0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void motor_control_task(void *pvParameters)
{
    char response[64];
    float current_angle = 0.0f;
    // 等待UART稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 1. 基础配置
    ESP_LOGI("foc", "步骤1: 配置电机A");

    // 先发送简单的命令测试通信
    ESP_LOGI("foc", "测试通信...");
    esp_err_t ret = motor_send_and_wait('A', "S0", response, sizeof(response), 500);
    if (ret == ESP_OK)
    {
        ESP_LOGI("foc", "通信测试成功: %s", response);
    }
    else
    {
        ESP_LOGW("foc", "通信测试失败，继续尝试配置...");
    }

    while (1)
    {
        // 2. 速度模式测试
        ESP_LOGI("foc", "步骤2: 速度模式 (100°/s, 2秒)");

        motor_send_cmd('A', "V"); // 进入速度模式
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_send_cmd('A', "100"); // 设置速度
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_send_cmd('A', "E1"); // 使能
        vTaskDelay(pdMS_TO_TICKS(2000));

        motor_send_cmd('A', "E0"); // 失能
        vTaskDelay(pdMS_TO_TICKS(500));

        // 3. 位置模式测试
        ESP_LOGI("foc", "步骤3: 位置模式 (45度)");

        motor_send_cmd('A', "P"); // 进入位置模式
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_send_cmd('A', "45"); // 设置目标位置
        vTaskDelay(pdMS_TO_TICKS(100));

        motor_send_cmd('A', "E1"); // 使能
        vTaskDelay(pdMS_TO_TICKS(1500));

        motor_send_cmd('A', "E0"); // 失能
        vTaskDelay(pdMS_TO_TICKS(500));

        // 4. 查询角度
        ESP_LOGI("foc", "步骤4: 查询当前角度");

        for (int retry = 0; retry < 3; retry++)
        {
            memset(response, 0, sizeof(response));

            if (motor_send_and_wait('A', "S0", response, sizeof(response), 300) == ESP_OK)
            {
                ESP_LOGI("foc", "收到角度响应: '%s'", response);

                if (motor_parse_angle_response(response, &current_angle) == ESP_OK)
                {
                    ESP_LOGI("foc", "解析成功: %.2f 度", current_angle);
                    break;
                }
                else
                {
                    ESP_LOGW("foc", "解析失败，原始数据: %s", response);
                }
            }
            else
            {
                ESP_LOGW("foc", "查询角度超时，重试 %d/3", retry + 1);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
    }
}
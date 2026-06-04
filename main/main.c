#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 步进电机 GPIO 定义
#define DIR_GPIO        GPIO_NUM_4
#define STEP_GPIO       GPIO_NUM_5
#define MS2_GPIO        GPIO_NUM_6
#define MS1_GPIO        GPIO_NUM_7
#define EN_GPIO         GPIO_NUM_15

#define STEP_TIMER          LEDC_TIMER_0
#define STEP_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define STEP_CHANNEL        LEDC_CHANNEL_0
#define STEP_RESOLUTION     LEDC_TIMER_12_BIT
#define STEP_DUTY           2048
#define STEP_FREQ_HZ        5000

//===== 减速比1:6 单圈参数 =====
#define PULSES_PER_REV      76800
#define RUN_TIME_MS         15360
//=============================

#define UART_PORT           UART_NUM_0
#define UART_BUF_SIZE       1024

static void uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_PORT, &uart_cfg);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0);
}

static void motor_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_GPIO) | (1ULL << MS2_GPIO) | (1ULL << MS1_GPIO) | (1ULL << EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(EN_GPIO, 1);
    gpio_set_level(MS1_GPIO, 0);
    gpio_set_level(MS2_GPIO, 1);
    gpio_set_level(DIR_GPIO, 0);
}

static void motor_pwm_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = STEP_SPEED_MODE,
        .timer_num  = STEP_TIMER,
        .duty_resolution = STEP_RESOLUTION,
        .freq_hz    = STEP_FREQ_HZ,
        .clk_cfg    = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t channel_cfg = {
        .gpio_num   = STEP_GPIO,
        .speed_mode = STEP_SPEED_MODE,
        .channel    = STEP_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = STEP_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&channel_cfg);
}

static void motor_run_one_revolution(void) {
    printf("===== 开始旋转：1圈 减速1:6 =====\n");
    gpio_set_level(EN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    ledc_set_duty(STEP_SPEED_MODE, STEP_CHANNEL, STEP_DUTY);
    ledc_update_duty(STEP_SPEED_MODE, STEP_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(RUN_TIME_MS));

    ledc_set_duty(STEP_SPEED_MODE, STEP_CHANNEL, 0);
    ledc_update_duty(STEP_SPEED_MODE, STEP_CHANNEL);
    gpio_set_level(EN_GPIO, 1);

    printf("===== 一圈结束停机 =====\n\n");
}

void app_main(void)
{
    uart_init();
    motor_gpio_init();
    motor_pwm_init();

    uint8_t data_buf[128];
    printf("发送ok运行一圈(1:6减速)\n");
    while (1) {
        int len = uart_read_bytes(UART_PORT, data_buf, sizeof(data_buf)-1, pdMS_TO_TICKS(50));
        if (len > 0) {
            data_buf[len] = '\0';
            if (strstr((char*)data_buf, "ok") != NULL) {
                motor_run_one_revolution();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
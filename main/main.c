#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <inttypes.h>

// 步进电机 GPIO 定义
#define DIR_GPIO        GPIO_NUM_4
#define STEP_GPIO       GPIO_NUM_5
#define MS2_GPIO        GPIO_NUM_6
#define MS1_GPIO        GPIO_NUM_7
#define EN_GPIO         GPIO_NUM_15

#define STEP_TIMER          LEDC_TIMER_0
#define STEP_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define STEP_CHANNEL        LEDC_CHANNEL_0
#define STEP_RESOLUTION     LEDC_TIMER_11_BIT
#define STEP_DUTY           1024

volatile uint8_t g_motor_run = 0;

uint32_t g_step_freq = 10000; // 默认10kHz
uint8_t g_auto_cycle = 0;     // 自动循环标志

#define UART_PORT           UART_NUM_0
#define CMD_BUF_LEN         64

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
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
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
        .freq_hz    = g_step_freq,
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

static void set_microstep(uint32_t step)
{
    switch(step)
    {
        case 8:
            gpio_set_level(MS1_GPIO,0);
            gpio_set_level(MS2_GPIO,0);
            printf("SET MICROSTEP: 1/8\n");
            break;
        case 16:
            gpio_set_level(MS1_GPIO,1);
            gpio_set_level(MS2_GPIO,0);
            printf("SET MICROSTEP: 1/16\n");
            break;
        case 32:
            gpio_set_level(MS1_GPIO,0);
            gpio_set_level(MS2_GPIO,1);
            printf("SET MICROSTEP: 1/32\n");
            break;
        case 64:
            gpio_set_level(MS1_GPIO,1);
            gpio_set_level(MS2_GPIO,1);
            printf("SET MICROSTEP: 1/64\n");
            break;
        default:
            printf("ERROR! Only s8 s16 s32 s64 available\n");
            break;
    }
}

static void print_command_menu(void) {
    printf("\n===== COMMAND =====\n");
    printf("on     -> AUTO CYCLE (500ms forward -> 500ms reverse)\n");
    printf("run    -> KEEP ROTATING\n");
    printf("off    -> STOP\n");
    printf("fX     -> Set Freq(X kHz)\n");
    printf("MIN:1kHz MAX:39kHz\n");
    printf("s8/s16/s32/s64 -> Set Microstep\n");
    printf("clear  -> Clear screen and show this menu\n");
    printf("===================\n");
}

void set_freq_auto(uint32_t freq_khz) {
    uint32_t hz = freq_khz * 1000;
    if (hz < 1000) hz = 1000;
    if (hz > 39000) hz = 39000;

    g_step_freq = hz;
    ledc_set_freq(STEP_SPEED_MODE, STEP_TIMER, g_step_freq);
    printf("FREQ SET OK: %" PRIu32 " Hz (%" PRIu32 " kHz)\n", g_step_freq, freq_khz);
}

// 运行指定方向 ms 时间
void run_ms(uint8_t dir, uint32_t ms) {
    gpio_set_level(DIR_GPIO, dir);
    gpio_set_level(EN_GPIO, 0);
    ledc_set_duty(STEP_SPEED_MODE, STEP_CHANNEL, STEP_DUTY);
    ledc_update_duty(STEP_SPEED_MODE, STEP_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(ms));

    if (!g_motor_run) {
        ledc_set_duty(STEP_SPEED_MODE, STEP_CHANNEL, 0);
        ledc_update_duty(STEP_SPEED_MODE, STEP_CHANNEL);
        gpio_set_level(EN_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// 自动循环任务
void auto_cycle_task(void *pv) {
    while(1) {
        if(g_auto_cycle) {
            run_ms(0, 500);
            
            if(!g_auto_cycle) continue;
            
            run_ms(1, 500);
        } else {
            vTaskDelay(10);
        }
    }
}

void motor_start(void) {
    g_motor_run = 0;
    g_auto_cycle = 1;
    printf("AUTO CYCLE START\n");
}

void motor_run(void) {
    g_auto_cycle = 0;
    g_motor_run = 1;
    gpio_set_level(DIR_GPIO, 0);
    gpio_set_level(EN_GPIO, 0);
    ledc_set_duty(STEP_SPEED_MODE, STEP_CHANNEL, STEP_DUTY);
    ledc_update_duty(STEP_SPEED_MODE, STEP_CHANNEL);
    printf("KEEP ROTATING START\n");
}

void motor_stop(void) {
    g_auto_cycle = 0;
    g_motor_run = 0;
    printf("MOTOR STOP\n");
    
    ledc_set_duty(STEP_SPEED_MODE, STEP_CHANNEL, 0);
    ledc_update_duty(STEP_SPEED_MODE, STEP_CHANNEL);
    gpio_set_level(EN_GPIO, 1);
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(300));

    uart_init();
    motor_gpio_init();
    motor_pwm_init();

    xTaskCreate(auto_cycle_task, "auto_task", 4096, NULL, 5, NULL);

    uint8_t dummy;
    while (uart_read_bytes(UART_PORT, &dummy, 1, 0) > 0);

    uint8_t cmd_buf[CMD_BUF_LEN] = {0};
    uint16_t idx = 0;
    uint8_t ch;

    print_command_menu();

    while (1) {
        if (uart_read_bytes(UART_PORT, &ch, 1, 0) > 0) {
            if (ch == 0x08 || ch == 0x7F) {
                if (idx > 0) {
                    idx--;
                    cmd_buf[idx] = 0;
                    uart_write_bytes(UART_PORT, "\b \b", 3);
                }
                continue;
            }
            uart_write_bytes(UART_PORT, (const char*)&ch, 1);

            if (ch == '\r' || ch == '\n') {
                if (idx > 0) {
                    cmd_buf[idx] = '\0';
                    printf("\n");

                    if (strcmp((char*)cmd_buf, "on") == 0) {
                        motor_start();
                    } else if (strcmp((char*)cmd_buf, "run") == 0) {
                        motor_run();
                    } else if (strcmp((char*)cmd_buf, "off") == 0) {
                        motor_stop();
                    } else if (strcmp((char*)cmd_buf, "clear") == 0) {
                        printf("\033[2J\033[H");
                        print_command_menu();
                    } else if (cmd_buf[0] == 'f') {
                        uint32_t khz = atoi((char*)cmd_buf + 1);
                        set_freq_auto(khz);
                    } else if (cmd_buf[0] == 's') {
                        uint32_t st = atoi((char*)cmd_buf + 1);
                        set_microstep(st);
                    }

                    idx = 0;
                    memset(cmd_buf, 0, CMD_BUF_LEN);
                }
            }
            else if (idx < CMD_BUF_LEN - 1) {
                cmd_buf[idx++] = ch;
            }
        }
        vTaskDelay(1);
    }
}

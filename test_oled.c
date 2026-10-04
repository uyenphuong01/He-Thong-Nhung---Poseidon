#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

/* PIN */
#define OLED_SDA_GPIO       5
#define OLED_SCL_GPIO       4
#define OLED_I2C_ADDRESS    0x3C
#define OLED_COLS           21              /* 128 px / 6 px mỗi ký tự */

#define DHT22_GPIO          6
#define LIGHT_ADC_CHANNEL   ADC_CHANNEL_0   
#define LED_GPIO            8
#define LED_ON_LEVEL        1              
#define SERVO_GPIO          7

/* ================= NGƯỠNG ĐIỀU KHIỂN =================
 * Bật khi < ngưỡng, chỉ tắt khi >= ngưỡng + trễ (hysteresis) */
#define HUMIDITY_THRESHOLD  80.0f
#define HUMIDITY_HYST       5.0f
#define LIGHT_THRESHOLD     30              /* tính theo % */
#define LIGHT_HYST          5

#define SERVO_CLOSED_ANGLE  0
#define SERVO_OPEN_ANGLE    90

/* ================= CHU KỲ & ƯU TIÊN ================= */
#define DHT_PERIOD_MS       2500            /* DHT22 cần >= 2 s giữa 2 lần đọc */
#define LIGHT_PERIOD_MS     2000
#define SEND_TIMEOUT_MS     100

#define PRIO_SENSOR         2               /* periodic, phần lớn thời gian Blocked */
#define PRIO_BRAIN          3               /* hướng sự kiện: chờ dataQueue */
#define PRIO_ACTUATOR       4               /* hướng sự kiện: chờ queue set */
#define TASK_STACK_BYTES    4096            /* ESP-IDF tính stack theo BYTE */

#define SERVO_FREQ          50
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL       LEDC_CHANNEL_0

static const char *TAG = "POSEIDON_SYSTEM";

typedef enum { SENSOR_TEMP = 1, SENSOR_HUMIDITY, SENSOR_LIGHT } SensorID;

typedef struct { SensorID id; float value; } SensorData;   /* dataQueue */
typedef struct { int command; } PumpCommand;               /* 1 = mở van, 0 = đóng */
typedef struct { int command; } LedCommand;                /* 1 = bật, 0 = tắt */

static QueueHandle_t    dataQueue;
static QueueHandle_t    pumpQueue;
static QueueHandle_t    ledQueue;
static QueueSetHandle_t actuatorQueueSet;

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t oled_dev = NULL;
static adc_oneshot_unit_handle_t adc_handle;
static portMUX_TYPE dht_mux = portMUX_INITIALIZER_UNLOCKED;

/* OLED LOW LEVEL */
static esp_err_t oled_write_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};
    return i2c_master_transmit(oled_dev, data, sizeof(data), 1000);
}

static esp_err_t oled_write_data(const uint8_t *data, size_t len)
{
    uint8_t buffer[129];
    if (len > 128) return ESP_ERR_INVALID_SIZE;
    buffer[0] = 0x40;
    memcpy(&buffer[1], data, len);
    return i2c_master_transmit(oled_dev, buffer, len + 1, 1000);
}

static void oled_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_I2C_ADDRESS,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &oled_dev));

    static const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof(init_seq); i++) oled_write_cmd(init_seq[i]);
}

static void oled_set_cursor(uint8_t x, uint8_t page)
{
    oled_write_cmd(0xB0 + page);
    oled_write_cmd(0x00 + (x & 0x0F));
    oled_write_cmd(0x10 + ((x >> 4) & 0x0F));
}

static void oled_clear(void)
{
    uint8_t blank[128] = {0};
    for (uint8_t page = 0; page < 8; page++) {
        oled_set_cursor(0, page);
        oled_write_data(blank, sizeof(blank));
    }
}

/* FONT 5x7 */
static const uint8_t font_digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
};
static const uint8_t font_A[5] = {0x7E,0x11,0x11,0x11,0x7E};
static const uint8_t font_C[5] = {0x3E,0x41,0x41,0x41,0x22};
static const uint8_t font_D[5] = {0x7F,0x41,0x41,0x22,0x1C};
static const uint8_t font_E[5] = {0x7F,0x49,0x49,0x49,0x41};
static const uint8_t font_F[5] = {0x7F,0x09,0x09,0x09,0x01};
static const uint8_t font_H[5] = {0x7F,0x08,0x08,0x08,0x7F};
static const uint8_t font_L[5] = {0x7F,0x40,0x40,0x40,0x40};
static const uint8_t font_N[5] = {0x7F,0x06,0x18,0x60,0x7F};
static const uint8_t font_O[5] = {0x3E,0x41,0x41,0x41,0x3E};
static const uint8_t font_P[5] = {0x7F,0x09,0x09,0x09,0x06};
static const uint8_t font_S[5] = {0x46,0x49,0x49,0x49,0x31};
static const uint8_t font_T[5] = {0x01,0x01,0x7F,0x01,0x01};
static const uint8_t font_V[5] = {0x1F,0x60,0x40,0x60,0x1F};
static const uint8_t font_colon[5]   = {0x00,0x36,0x36,0x00,0x00};
static const uint8_t font_dot[5]     = {0x00,0x60,0x60,0x00,0x00};
static const uint8_t font_percent[5] = {0x62,0x64,0x08,0x13,0x23};
static const uint8_t font_minus[5]   = {0x08,0x08,0x08,0x08,0x08};
static const uint8_t font_space[5]   = {0x00,0x00,0x00,0x00,0x00};

static const uint8_t *get_font(char c)
{
    if (c >= '0' && c <= '9') return font_digits[c - '0'];
    switch (c) {
        case 'A': return font_A;   case 'C': return font_C;
        case 'D': return font_D;   case 'E': return font_E;
        case 'F': return font_F;   case 'H': return font_H;
        case 'L': return font_L;   case 'N': return font_N;
        case 'O': return font_O;   case 'P': return font_P;
        case 'S': return font_S;   case 'T': return font_T;
        case 'V': return font_V;
        case ':': return font_colon;
        case '.': return font_dot;
        case '%': return font_percent;
        case '-': return font_minus;
        default:  return font_space;
    }
}

static void oled_print(const char *text)
{
    uint8_t buf[OLED_COLS * 6];
    size_t n = 0;
    while (*text && n + 6 <= sizeof(buf)) {
        memcpy(&buf[n], get_font(*text++), 5);
        buf[n + 5] = 0x00;
        n += 6;
    }
    if (n) oled_write_data(buf, n);
}

static void oled_print_line(uint8_t page, const char *text)
{
    char buf[OLED_COLS + 1];
    snprintf(buf, sizeof(buf), "%-*s", OLED_COLS, text);
    oled_set_cursor(0, page);
    oled_print(buf);
}

static void oled_show_status(bool valve_open, bool led_on)
{
    char line[32];
    snprintf(line, sizeof(line), "VALVE:%s LED:%s",
             valve_open ? "ON" : "OFF", led_on ? "ON" : "OFF");
    oled_print_line(6, line);
}

/* ADC / LIGHT */
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LIGHT_ADC_CHANNEL, &config));
}

static int light_read_raw(void)         /* trung bình 8 mẫu để giảm nhiễu */
{
    int sum = 0, raw = 0;
    for (int i = 0; i < 8; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, LIGHT_ADC_CHANNEL, &raw));
        sum += raw;
    }
    return sum / 8;
}

static int light_percentage(int raw)
{
    const int LIGHT_DARK = 4095;
    const int LIGHT_BRIGHT = 500;
    int percent = (LIGHT_DARK - raw) * 100 / (LIGHT_DARK - LIGHT_BRIGHT);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}

/*  DHT22 */
/* Chờ đến khi chân đạt mức `level`; trả về thời gian đã chờ (µs) */
static bool dht22_wait_level(int level, uint32_t timeout_us, uint32_t *elapsed_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(DHT22_GPIO) != level) {
        if ((esp_timer_get_time() - start) > (int64_t)timeout_us) return false;
    }
    if (elapsed_us) *elapsed_us = (uint32_t)(esp_timer_get_time() - start);
    return true;
}

static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};
    bool ok = true;

    /* Start signal: kéo thấp >= 1 ms */
    gpio_set_direction(DHT22_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT22_GPIO, 0);
    esp_rom_delay_us(1200);

    /* Từ đây đến hết 40 bit: timing tính bằng µs, KHÔNG được để scheduler chen ngang */
    portENTER_CRITICAL(&dht_mux);

    gpio_set_level(DHT22_GPIO, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT22_GPIO, GPIO_MODE_INPUT);

    if (!dht22_wait_level(0, 100, NULL) ||
        !dht22_wait_level(1, 100, NULL) ||
        !dht22_wait_level(0, 100, NULL)) {
        ok = false;
    }

    for (int i = 0; ok && i < 40; i++) {
        uint32_t high_us = 0;
        if (!dht22_wait_level(1, 100, NULL) ||          /* hết đoạn thấp ~50 µs */
            !dht22_wait_level(0, 100, &high_us)) {      /* đo độ rộng xung cao */
            ok = false;
            break;
        }
        data[i / 8] <<= 1;
        if (high_us > 45) data[i / 8] |= 1;             /* ~27 µs = 0, ~70 µs = 1 */
    }

    portEXIT_CRITICAL(&dht_mux);

    if (!ok) return false;
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return false;

    *humidity = (((uint16_t)data[0] << 8) | data[1]) / 10.0f;

    uint16_t t_raw = ((uint16_t)data[2] << 8) | data[3];
    if (t_raw & 0x8000) {
        t_raw &= 0x7FFF;
        *temperature = -(t_raw / 10.0f);
    } else {
        *temperature = t_raw / 10.0f;
    }
    return true;
}

static void dht22_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << DHT22_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

/* LED */
static void led_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(LED_GPIO, !LED_ON_LEVEL);
}

/* SERVO */
static void servo_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_CHANNEL,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

static void servo_set_angle(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    float pulse_us = 500.0f + (2000.0f * angle) / 180.0f;         /* 0.5 – 2.5 ms */
    uint32_t duty = (uint32_t)((pulse_us / 20000.0f) * (1 << 14)); /* chu kỳ 20 ms */

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL));
}

/* GỬI GÓI VÀO dataQueue */
/* Block tối đa SEND_TIMEOUT_MS nếu queue đầy (mục 5.2.4), không block vô hạn */
static bool send_packet(SensorID id, float value)
{
    SensorData packet = { .id = id, .value = value };

    if (xQueueSendToBack(dataQueue, &packet, pdMS_TO_TICKS(SEND_TIMEOUT_MS)) != pdPASS) {
        ESP_LOGW(TAG, "dataQueue full, packet (id=%d) dropped", (int)id);
        return false;
    }
    return true;
}

/* TASK 1: DHT (sender 1) */
static void dht_task(void *pvParameters)
{
    /* DHT22 cần ~2 s sau khi cấp nguồn mới đọc được */
    vTaskDelay(pdMS_TO_TICKS(2000));

    TickType_t last_wake = xTaskGetTickCount();     /* khởi tạo đúng 1 lần (mục 4.7.5) */

    for (;;)
    {
        float temperature, humidity;

        if (dht22_read(&temperature, &humidity))
        {
            /* 1 lần đọc DHT22 cho 2 giá trị -> gửi 2 gói vào cùng dataQueue */
            send_packet(SENSOR_TEMP, temperature);
            send_packet(SENSOR_HUMIDITY, humidity);

            ESP_LOGI(TAG, "[DHT]   T = %.1f C, H = %.1f %%", temperature, humidity);
        }
        else
        {
            ESP_LOGW(TAG, "[DHT]   read failed");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DHT_PERIOD_MS));
    }
}

/* TASK 2: LIGHT (sender 2) */
static void light_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        int raw = light_read_raw();

        /* Gửi giá trị ADC thô; brain tự quy đổi ra % để so ngưỡng */
        if (send_packet(SENSOR_LIGHT, (float)raw)) {
            ESP_LOGI(TAG, "[LIGHT] ADC = %d (%d %%)", raw, light_percentage(raw));
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LIGHT_PERIOD_MS));
    }
}

/* TASK 3: DISPLAY & BRAIN (receiver) */
static void display_brain_task(void *pvParameters)
{
    SensorData data;
    bool valve_open = false;
    bool led_on = false;
    char line[32];

    for (;;)
    {
        /* Blocked cho tới khi có gói: không tốn CPU */
        if (xQueueReceive(dataQueue, &data, portMAX_DELAY) != pdPASS) continue;

        switch (data.id)    /* id cho biết gói đến từ đâu */
        {
        case SENSOR_TEMP:
        {
            snprintf(line, sizeof(line), "T:%.1fC", data.value);
            oled_print_line(0, line);
            break;
        }

        case SENSOR_HUMIDITY:
        {
            snprintf(line, sizeof(line), "H:%.1f%%", data.value);
            oled_print_line(2, line);

            bool want_open = valve_open;
            if (data.value < HUMIDITY_THRESHOLD)
                want_open = true;
            else if (data.value >= HUMIDITY_THRESHOLD + HUMIDITY_HYST)
                want_open = false;

            if (want_open != valve_open)        /* chỉ gửi lệnh khi trạng thái đổi */
            {
                PumpCommand cmd = { .command = want_open ? 1 : 0 };
                if (xQueueSend(pumpQueue, &cmd, pdMS_TO_TICKS(SEND_TIMEOUT_MS)) == pdPASS) {
                    valve_open = want_open;
                    oled_show_status(valve_open, led_on);
                    ESP_LOGI(TAG, "[BRAIN] H=%.1f -> %s valve", data.value,
                             valve_open ? "OPEN" : "CLOSE");
                } else {
                    ESP_LOGW(TAG, "[BRAIN] pumpQueue full");
                }
            }
            break;
        }

        case SENSOR_LIGHT:
        {
            int raw = (int)data.value;                  /* giá trị ADC đọc được */
            int percent = light_percentage(raw);        /* quy đổi ra % */

            snprintf(line, sizeof(line), "L:%d%% ADC:%d", percent, raw);
            oled_print_line(4, line);

            bool want_on = led_on;
            if (percent < LIGHT_THRESHOLD)
                want_on = true;
            else if (percent >= LIGHT_THRESHOLD + LIGHT_HYST)
                want_on = false;

            if (want_on != led_on)
            {
                LedCommand cmd = { .command = want_on ? 1 : 0 };
                if (xQueueSend(ledQueue, &cmd, pdMS_TO_TICKS(SEND_TIMEOUT_MS)) == pdPASS) {
                    led_on = want_on;
                    oled_show_status(valve_open, led_on);
                    ESP_LOGI(TAG, "[BRAIN] ADC=%d (%d%%) -> LED %s", raw, percent,
                             led_on ? "ON" : "OFF");
                } else {
                    ESP_LOGW(TAG, "[BRAIN] ledQueue full");
                }
            }
            break;
        }

        default:
            break;
        }
    }
}

/* TASK 4: ACTUATOR (Queue Set) */
static void actuator_task(void *pvParameters)
{
    for (;;)
    {
        /* Một task chờ đồng thời 2 queue (mục 5.6) */
        QueueSetMemberHandle_t active = xQueueSelectFromSet(actuatorQueueSet, portMAX_DELAY);

        if (active == (QueueSetMemberHandle_t)pumpQueue)
        {
            PumpCommand cmd;
            /* Handle đã được trả về -> queue chắc chắn có dữ liệu, block time = 0 */
            if (xQueueReceive(pumpQueue, &cmd, 0) == pdPASS) {
                servo_set_angle(cmd.command ? SERVO_OPEN_ANGLE : SERVO_CLOSED_ANGLE);
                ESP_LOGI(TAG, "[ACT]   VALVE %s", cmd.command ? "OPEN" : "CLOSED");
            }
        }
        else if (active == (QueueSetMemberHandle_t)ledQueue)
        {
            LedCommand cmd;
            if (xQueueReceive(ledQueue, &cmd, 0) == pdPASS) {
                gpio_set_level(LED_GPIO, cmd.command ? LED_ON_LEVEL : !LED_ON_LEVEL);
                ESP_LOGI(TAG, "[ACT]   LED %s", cmd.command ? "ON" : "OFF");
            }
        }
    }
}

static void create_task(TaskFunction_t fn, const char *name, UBaseType_t prio)
{
    if (xTaskCreate(fn, name, TASK_STACK_BYTES, NULL, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task %s (not enough heap)", name);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 FreeRTOS Smart System");

    oled_init();
    oled_clear();
    adc_init();
    dht22_init();
    led_init();
    servo_init();
    servo_set_angle(SERVO_CLOSED_ANGLE);

    /* PHẢI tạo hết queue TRƯỚC khi tạo task: task ưu tiên cao hơn app_main
     * sẽ preempt app_main ngay khi được tạo (giống Example 4.9). */
    dataQueue = xQueueCreate(10, sizeof(SensorData));
    pumpQueue = xQueueCreate(5,  sizeof(PumpCommand));
    ledQueue  = xQueueCreate(5,  sizeof(LedCommand));
    actuatorQueueSet = xQueueCreateSet(5 + 5);   /* = tổng độ dài các queue thành viên */

    if (!dataQueue || !pumpQueue || !ledQueue || !actuatorQueueSet) {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

    xQueueAddToSet(pumpQueue, actuatorQueueSet);   /* queue phải còn rỗng khi thêm */
    xQueueAddToSet(ledQueue,  actuatorQueueSet);

    oled_print_line(0, "T:--.-C");
    oled_print_line(2, "H:--.-%");
    oled_print_line(4, "L:---% ADC:----");
    oled_show_status(false, false);

    create_task(dht_task,           "DHT",      PRIO_SENSOR);
    create_task(light_task,         "Light",    PRIO_SENSOR);
    create_task(display_brain_task, "Brain",    PRIO_BRAIN);
    create_task(actuator_task,      "Actuator", PRIO_ACTUATOR);

    ESP_LOGI(TAG, "All tasks started");
    vTaskDelete(NULL);      /* Idle task sẽ dọn TCB + stack của app_main */
}
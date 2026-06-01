#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define UART_PORT UART_NUM_2
#define UART_TX GPIO_NUM_17
#define UART_RX GPIO_NUM_16

#define ADC_UNIT_USED ADC_UNIT_1
#define ADC_CH ADC_CHANNEL_6

#define MCP4132_WIPER0 0x00
#define MCP4132_CMD_WRITE 0x00
#define MCP4132_CMD_READ 0x03

#define SAMPLE_PERIOD_US 1000

#define UMBRAL_ALTO_MV 1400
#define UMBRAL_BAJO_MV 900

#define RAB_OHMS 10000.0f
#define RW_OHMS 75.0f
#define CAP_F 0.0000001f

static spi_device_handle_t mcp4132;
static adc_oneshot_unit_handle_t adc_handle;
static int wiper_actual = -1;

void mcp4132_write_register(uint8_t address, uint16_t value) {
    uint8_t tx[2];

    address &= 0x0F;
    value &= 0x01FF;

    tx[0] = (address << 4) | (MCP4132_CMD_WRITE << 2) | ((value >> 8) & 0x03);
    tx[1] = value & 0xFF;

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;

    spi_device_transmit(mcp4132, &t);
}

uint16_t mcp4132_read_register(uint8_t address) {
    uint8_t tx[2];
    uint8_t rx[2];

    address &= 0x0F;

    tx[0] = (address << 4) | (MCP4132_CMD_READ << 2);
    tx[1] = 0x00;

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    spi_device_transmit(mcp4132, &t);

    return rx[1] & 0x7F;
}

void mcp4132_set_wiper(uint16_t n) {
    if (n > 128) {
        n = 128;
    }

    mcp4132_write_register(MCP4132_WIPER0, n);
}

void mcp4132_set_cutoff_frequency(float fc_hz) {
    if (fc_hz <= 0.0f) {
        return;
    }

    float r_obj = 1.0f / (2.0f * M_PI * fc_hz * CAP_F);
    float n_float = ((r_obj - RW_OHMS) * 128.0f) / RAB_OHMS;

    int n = (int)roundf(n_float);

    if (n < 0) {
        n = 0;
    }

    if (n > 128) {
        n = 128;
    }

    mcp4132_set_wiper(n);
}

extern "C" void app_main(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    adc_oneshot_unit_init_cfg_t adc_unit_cfg = {};
    adc_unit_cfg.unit_id = ADC_UNIT_USED;
    adc_oneshot_new_unit(&adc_unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t adc_chan_cfg = {};
    adc_chan_cfg.bitwidth = ADC_BITWIDTH_12;
    adc_chan_cfg.atten = ADC_ATTEN_DB_12;
    adc_oneshot_config_channel(adc_handle, ADC_CH, &adc_chan_cfg);

    timer_config_t timer_config = {};
    timer_config.divider = 80;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_PAUSE;
    timer_config.alarm_en = TIMER_ALARM_DIS;
    timer_config.auto_reload = TIMER_AUTORELOAD_EN;

    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);

    uint64_t last = 0;
    uint64_t now = 0;

    char msg[80];

    while (1) {
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &now);

        if (now - last >= SAMPLE_PERIOD_US) {
            last = now;
            int raw = 0;
            adc_oneshot_read(adc_handle, ADC_CH, &raw);
            int mv = (raw * 3300) / 4095;
            if (mv > UMBRAL_ALTO_MV && wiper_actual != 95) {
                wiper_actual = 95;
                mcp4132_set_wiper(95);

                snprintf(msg, sizeof(msg), "WIPER,N=95,V=%d mV\r\n", mv);
                uart_write_bytes(UART_PORT, msg, strlen(msg));
            }

            else if (mv < UMBRAL_BAJO_MV && wiper_actual != 42) {
                wiper_actual = 42;
                mcp4132_set_wiper(42);

                snprintf(msg, sizeof(msg), "WIPER,N=4,V=%d mV\r\n", mv);
                uart_write_bytes(UART_PORT, msg, strlen(msg));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
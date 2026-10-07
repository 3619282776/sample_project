#include "uart_stream.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"

#define UART_STREAM_NUM   UART_NUM_0
#define UART_STREAM_BAUD  921600        // 16 kHz * 2 字节/样本 ≈ 32 KB/s，921600 余量充足
#define UART_STREAM_TX_BUF 16384        // 发送环形缓冲，覆盖突发写入
#define UART_STREAM_RX_BUF 256          // 驱动要求 rx_buf > UART_FIFO_LEN(127)，否则报 "rx buffer length error"
#define UART_STREAM_TX_GPIO 1           // UART0 默认 TX，接板载 USB 转串口的 RXD
#define UART_STREAM_RX_GPIO 3           // UART0 默认 RX（板载 USB 转串口 TXD）；LCD RST 已接 VCC，GPIO3 空闲，双向可用

#define FRAME_MAGIC0      0xAA
#define FRAME_MAGIC1      0x55
#define FRAME_MAX_SAMPLES 2048          // channels * frames 上限，防止溢出栈/缓冲

static const char *TAG = "uart_stream";

// 组帧缓冲区：帧头(2) + 通道(1) + 帧数(2) + 样本(F) + 校验(1)
static uint8_t s_frame[5 + FRAME_MAX_SAMPLES * 2 + 1];

// 日志重定向：安装 UART 驱动后，ESP_LOG 也必须走驱动，否则会直接写 UART 寄存器与波形流交错。
// 这样日志与波形共享同一个环形缓冲，顺序不会乱；波特率也统一为 921600（monitor 需 -b 921600）。
static int stream_log_vprintf(const char *fmt, va_list args)
{
    char buf[128];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len < 0) {
        return len;
    }
    if (len > (int)sizeof(buf) - 1) {
        len = sizeof(buf) - 1;
    }
    return uart_write_bytes(UART_STREAM_NUM, buf, len);
}

esp_err_t uart_stream_init(void)
{
    // 上行发波形、下行收时间帧，双向共用 UART0；RX 缓冲 ≥128 供接收使用。
    esp_err_t err = uart_driver_install(UART_STREAM_NUM, UART_STREAM_RX_BUF, UART_STREAM_TX_BUF, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    const uart_config_t cfg = {
        .baud_rate  = UART_STREAM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(UART_STREAM_NUM, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    // TX=GPIO1（板载 USB 转串口 RXD）；RX=GPIO3（板载 USB 转串口 TXD），双向通信
    err = uart_set_pin(UART_STREAM_NUM, UART_STREAM_TX_GPIO,
                       UART_STREAM_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_log_set_vprintf(stream_log_vprintf);
    ESP_LOGI(TAG, "stream UART ready on TX=GPIO%d RX=GPIO%d, %d baud", UART_STREAM_TX_GPIO, UART_STREAM_RX_GPIO, UART_STREAM_BAUD);
    return ESP_OK;
}

int uart_stream_send(const int16_t *samples, uint8_t channels, int frames)
{
    if (samples == NULL || channels == 0 || frames <= 0) {
        return -1;
    }
    int count = (int)channels * frames;
    if (count > FRAME_MAX_SAMPLES) {
        return -1;
    }

    // 帧格式：[0xAA 0x55][channels][frames_lo][frames_hi][int16 样本小端 ...][checksum]
    s_frame[0] = FRAME_MAGIC0;
    s_frame[1] = FRAME_MAGIC1;
    s_frame[2] = channels;
    s_frame[3] = (uint8_t)(frames & 0xFF);
    s_frame[4] = (uint8_t)((frames >> 8) & 0xFF);

    uint8_t *data = &s_frame[5];
    memcpy(data, samples, (size_t)count * sizeof(int16_t));

    // checksum = 帧头之后、校验位之前所有字节之和的低 8 位
    uint8_t cks = 0;
    for (int i = 2; i < 5 + count * 2; i++) {
        cks += s_frame[i];
    }
    s_frame[5 + count * 2] = cks;

    int total = 5 + count * 2 + 1;
    int written = uart_write_bytes(UART_STREAM_NUM, s_frame, total);
    return (written < 0) ? -1 : written;
}
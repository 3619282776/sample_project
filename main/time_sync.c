#include "time_sync.h"

#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled_display.h"

#define TIME_SYNC_UART    UART_NUM_0
#define TIME_MAGIC0       0x55
#define TIME_MAGIC1       0xAA
#define TIME_CMD          0x01
#define TIME_FRAME_LEN    11

static void time_sync_task(void *arg)
{
    uint8_t buf[32];
    int len = 0;

    for (;;) {
        int n = uart_read_bytes(TIME_SYNC_UART, buf + len, sizeof(buf) - len, pdMS_TO_TICKS(100));
        if (n <= 0) {
            continue;
        }
        len += n;

        int pos = 0;
        while (pos + TIME_FRAME_LEN <= len) {
            if (buf[pos] == TIME_MAGIC0 && buf[pos + 1] == TIME_MAGIC1 && buf[pos + 2] == TIME_CMD) {
                uint8_t cks = 0;
                for (int i = 2; i < TIME_FRAME_LEN - 1; i++) {
                    cks += buf[pos + i];
                }
                if (cks == buf[pos + TIME_FRAME_LEN - 1]) {
                    int year  = buf[pos + 3] | (buf[pos + 4] << 8);
                    int month = buf[pos + 5];
                    int day   = buf[pos + 6];
                    int hour  = buf[pos + 7];
                    int min   = buf[pos + 8];
                    int sec   = buf[pos + 9];
                    oled_show_time(OLED_0, year, month, day, hour, min, sec);
                    pos += TIME_FRAME_LEN;
                    continue;
                }
            }
            pos++;   // 非帧头或校验失败：跳过 1 字节重新同步
        }

        if (pos > 0) {
            memmove(buf, buf + pos, len - pos);
            len -= pos;
        }
    }
}

esp_err_t time_sync_start(void)
{
    if (xTaskCreate(time_sync_task, "time_sync", 2048, NULL, 2, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
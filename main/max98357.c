#include "max98357.h"
#include "audio_config.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "max98357";
static i2s_chan_handle_t s_tx_handle = NULL;

esp_err_t amp_i2s_init(void)
{
    // 使能脚有效时先拉低（关断），避免上电爆音
    if (AMP_SD_GPIO >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << AMP_SD_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
        };
        gpio_config(&io);
        gpio_set_level(AMP_SD_GPIO, 0);
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AMP_BCK_GPIO,
            .ws   = AMP_WS_GPIO,
            .dout = AMP_DIN_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_handle, NULL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err)); return err; }

    err = i2s_channel_init_std_mode(s_tx_handle, &std_cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "init std mode: %s", esp_err_to_name(err)); return err; }

    err = i2s_channel_enable(s_tx_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "enable: %s", esp_err_to_name(err)); return err; }

    // 时钟已启动后再使能功放
    if (AMP_SD_GPIO >= 0) {
        gpio_set_level(AMP_SD_GPIO, 1);
    }

    ESP_LOGI(TAG, "amp I2S TX ready");
    return ESP_OK;
}


esp_err_t amp_write(const int16_t *mono, int samples, uint32_t timeout_ms)
{
    if (samples <= 0) {
        return ESP_OK;
    }
    // MAX98357A 为单声道，声道选择因板型而异；把单声道复制到左右声道以规避歧义
    static int16_t stereo[AUDIO_BLOCK_FRAMES * 2];
    if (samples > AUDIO_BLOCK_FRAMES) {
        samples = AUDIO_BLOCK_FRAMES;
    }
    for (int i = 0; i < samples; i++) {
        stereo[i * 2]     = mono[i];
        stereo[i * 2 + 1] = mono[i];
    }
    size_t want = (size_t)samples * 2 * sizeof(int16_t);
    size_t written = 0;
    return i2s_channel_write(s_tx_handle, stereo, want, &written, timeout_ms);
}

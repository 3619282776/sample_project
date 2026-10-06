#include "inmp441_doa.h"
#include "audio_config.h"

#include <math.h>
#include "driver/i2s_std.h"
#include "esp_log.h"

#define PI_F 3.14159265358979323846f

static const char *TAG = "inmp441_doa";
static i2s_chan_handle_t s_rx_handle = NULL;

esp_err_t mic_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_GPIO,
            .ws   = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN_GPIO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err)); return err; }

    err = i2s_channel_init_std_mode(s_rx_handle, &std_cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "init std mode: %s", esp_err_to_name(err)); return err; }

    err = i2s_channel_enable(s_rx_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "enable: %s", esp_err_to_name(err)); return err; }

    ESP_LOGI(TAG, "mic I2S RX ready (%u Hz, stereo)", (unsigned)AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

int mic_i2s_read(int32_t *interleaved, int max_frames, uint32_t timeout_ms)
{
    size_t want = (size_t)max_frames * 2 * sizeof(int32_t);
    size_t got = 0;
    esp_err_t err = i2s_channel_read(s_rx_handle, interleaved, want, &got, timeout_ms);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "read error: %s", esp_err_to_name(err));
        return -1;
    }
    int samples = (int)(got / sizeof(int32_t));
    // INMP441 24bit 数据左对齐在 32bit 槽内，算术右移 8 位还原有符号样本
    for (int i = 0; i < samples; i++) {
        interleaved[i] >>= 8;
    }
    return samples / 2;  // 返回帧数
}

float doa_estimate_angle(const int32_t *l, const int32_t *r, int n)
{
    // 静音检测：平均幅值低于门限时认为没有有效声源，返回 NAN（无效方向）。
    // 这样安静环境下不会因底噪把方向算得乱跳。门限见 audio_config.h 的 DOA_MIN_AMPLITUDE。
    int64_t amp_sum = 0;
    for (int i = 0; i < n; i++) {
        amp_sum += (l[i] >= 0) ? (int64_t)l[i] : -(int64_t)l[i];
        amp_sum += (r[i] >= 0) ? (int64_t)r[i] : -(int64_t)r[i];
    }
    float mean_amp = (float)amp_sum / (float)(n * 2);
    if (mean_amp < DOA_MIN_AMPLITUDE) {
        return NAN;
    }

    // 小滞后范围内的时域互相关（GCC 的简化形式，无需 FFT）
    const int L = DOA_MAX_LAG;
    int64_t corr[2 * L + 1];
    for (int lag = -L; lag <= L; lag++) {
        int64_t acc = 0;
        for (int i = L; i < n - L; i++) {
            acc += (int64_t)l[i] * r[i + lag];
        }
        corr[lag + L] = acc;
    }

    int peak = 0;
    for (int k = 1; k <= 2 * L; k++) {
        if (corr[k] > corr[peak]) {
            peak = k;
        }
    }

    // 抛物线插值，获得亚采样精度的滞后
    float lag_f = (float)(peak - L);
    if (peak > 0 && peak < 2 * L) {
        float c0 = (float)corr[peak - 1];
        float c1 = (float)corr[peak];
        float c2 = (float)corr[peak + 1];
        float denom = c0 - 2.0f * c1 + c2;
        if (denom != 0.0f) {
            lag_f += 0.5f * (c0 - c2) / denom;
        }
    }

    // tau 为右麦相对左麦的延迟（秒）；声源在右侧时右麦先收到 -> 滞后为负
    float tau = lag_f / (float)AUDIO_SAMPLE_RATE;
    float sin_theta = -tau * SPEED_OF_SOUND / MIC_SPACING_M;
    if (sin_theta > 1.0f) sin_theta = 1.0f;
    if (sin_theta < -1.0f) sin_theta = -1.0f;
    return asinf(sin_theta) * 180.0f / PI_F;
}

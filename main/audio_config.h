#pragma once

// 全局音频参数（麦克风采集与功放播放共用同一采样率，保证 1:1 直通）
#define AUDIO_SAMPLE_RATE    16000   // 采样率 Hz（INMP441 支持 8k~48k）

// 每次读/写的帧数（1 帧 = 1 个采样点；对麦克风立体声而言即 L+R 两个样本）
#define AUDIO_BLOCK_FRAMES   256     // 16k 下约 16ms

// 方位估计（DOA）参数
#define DOA_WINDOW           1024    // 每通道相关窗口样本数（16k 下约 64ms）
#define DOA_MAX_LAG          8       // 最大相关滞后样本数（16k 下约对应 17cm 间距）
#define SPEED_OF_SOUND       343.0f  // 声速 m/s
#define MIC_SPACING_M        0.08f   // 两片 INMP441 物理间距（米），按实际摆放修改

// 回放环形缓冲（FreeRTOS StreamBuffer）大小，单位字节
#define PLAYBACK_BUF_BYTES   4096    // 约 128ms

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pc_waveform.py — 实时显示 ESP32 INMP441 麦克风波形（USB 串口透传）

帧协议（与 main/uart_stream.c 保持一致，小端）：
    [0xAA 0x55][channels u8][frames u16][int16 样本 x (channels*frames)][checksum u8]
    checksum = 帧头之后、校验位之前所有字节之和 mod 256

用法：
    python pc_waveform.py                   # 自动枚举串口
    python pc_waveform.py COM5              # 指定串口（默认 921600 / 16 kHz）
    python pc_waveform.py COM5 921600 16000
    python pc_waveform.py --selftest        # 编解码自测（无需硬件/串口）

依赖：pip install pyserial numpy pyqtgraph pyqt5
"""

import argparse
import collections
import datetime
import itertools
import sys
import threading
import time

import numpy as np

MAGIC = b"\xaa\x55"
FRAME_HDR = 5  # magic(2) + channels(1) + frames(2)

# 下行（PC -> ESP32）时间帧头，与上行波形帧的 0xAA 0x55 相反，用于区分方向
TIME_MAGIC = b"\x55\xaa"
TIME_CMD = 0x01
TIME_FRAME_LEN = 11


def enc_time_frame(dt):
    """把电脑时间编码成下行时间帧：帧头校验与 ESP 端 time_sync.c 保持一致（小端）。"""
    body = bytearray(TIME_MAGIC)
    body.append(TIME_CMD)
    body.append(dt.year & 0xFF)
    body.append((dt.year >> 8) & 0xFF)
    body.append(dt.month & 0xFF)
    body.append(dt.day & 0xFF)
    body.append(dt.hour & 0xFF)
    body.append(dt.minute & 0xFF)
    body.append(dt.second & 0xFF)
    body.append(sum(body[2:]) & 0xFF)
    return bytes(body)


class FrameParser:
    """从字节流中同步并解析波形帧；异常/校验失败的帧会被丢弃并重新同步。"""

    def __init__(self):
        self._buf = bytearray()
        self.frames = 0      # 成功解析的帧数
        self.dropped = 0     # 因校验失败丢弃的次数（用于观测链路质量）

    def feed(self, data):
        """输入原始字节，返回 [(samples_float, channels), ...]，samples 为深度 -1/1 的交错样本。"""
        self._buf.extend(data)
        out = []
        while True:
            idx = self._buf.find(MAGIC)
            if idx < 0:
                keep = min(1, len(self._buf))          # 保留末 1 字节，防止帧头跨包被截断
                if keep:
                    self._buf[:] = self._buf[-keep:]
                else:
                    self._buf.clear()
                return out
            if idx > 0:
                del self._buf[:idx]                    # 丢弃同步前的噪声字节

            if len(self._buf) < FRAME_HDR:
                return out
            ch = self._buf[2]
            fr = self._buf[3] | (self._buf[4] << 8)
            n = ch * fr
            if ch == 0 or fr == 0 or n > 4096 * 8:     # 非法帧头，丢弃 1 字节重同步
                del self._buf[0]
                self.dropped += 1
                continue

            total = FRAME_HDR + 2 * n + 1
            if len(self._buf) < total:
                return out

            frame = bytes(self._buf[:total])
            cks = sum(frame[2:FRAME_HDR + 2 * n]) & 0xFF
            if cks != frame[-1]:
                del self._buf[0]
                self.dropped += 1
                continue

            samples = np.frombuffer(frame[FRAME_HDR:FRAME_HDR + 2 * n], dtype="<i2")
            out.append((samples.astype(np.float32) * (1.0 / 32768.0), ch))
            self.frames += 1
            del self._buf[:total]


def enc_frame(samples, channels, frames):
    """与 ESP 端 uart_stream_send 相同的编码，供自测使用。samples 为 int16 交错样本。"""
    body = bytearray(MAGIC)
    body.append(channels & 0xFF)
    body.append(frames & 0xFF)
    body.append((frames >> 8) & 0xFF)
    body.extend(np.asarray(samples, dtype="<i2").tobytes())
    body.append(sum(body[2:]) & 0xFF)
    return bytes(body)


def _fail(msg, code=1):
    print(msg)
    try:
        input("按回车键退出...")
    except Exception:
        pass
    sys.exit(code)


def pick_port():
    import serial.tools.list_ports as lp
    ports = [p.device for p in lp.comports()]
    if not ports:
        print("未发现串口。请确认 ESP32 已通过 USB 连接。")
        return None
    if len(ports) == 1:
        return ports[0]
    print("检测到多个串口：")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")
    sel = input("请选择编号: ").strip()
    try:
        return ports[int(sel)]
    except (ValueError, IndexError):
        print("选择无效。")
        return None


def selftest():
    rng = np.random.default_rng(0)
    parser = FrameParser()

    # 1) 串口噪声 + 一帧真实数据，验证重同步能力
    parser.feed(b"garbage-noise-\x00\x01###" + enc_frame([1, -2, 3, -4, 5], 1, 5))

    # 2) 200 帧随机数据，每帧随机切成两段喂入，验证跨包/分包解析
    for _ in range(200):
        data = rng.integers(-32768, 32767, size=256, dtype=np.int16)
        blk = enc_frame(data, 1, 256)
        cut = int(rng.integers(1, len(blk)))
        parser.feed(blk[:cut])
        parser.feed(blk[cut:])

    assert parser.frames == 201, f"期望 201 帧，实际解析 {parser.frames}"
    print(f"帧同步/跨包自测通过：解析 {parser.frames} 帧，丢弃 {parser.dropped} 次")

    # 3) 数值往返：编码已知样本 -> 解析 -> 比对，确保两端协议一致
    data = np.array([100, -100, 2024, -2024, 32767, -32768], dtype=np.int16)
    res = FrameParser().feed(enc_frame(data, 1, len(data)))
    assert len(res) == 1 and res[0][1] == 1, "通道数不符"
    assert np.allclose(res[0][0], data.astype(np.float32) / 32768.0), "样本数值不符"
    print("数值往返自测通过")
    print("OK — 协议与解析器一致。")


FFT_N = 512          # 时频图每帧 FFT 长度（16kHz 下约 32ms，频率分辨率约 31Hz）
HOP = FFT_N // 2     # 帧移（hop size）

# 人声主要频段：下限覆盖基频（男 85~180Hz / 女 165~255Hz），上限覆盖语音谐波与 300~3400Hz 语音带宽
VOICE_FMIN = 60.0
VOICE_FMAX = 4000.0


def run_gui(port, baud, fs, window_sec, spec_sec=5.0):
    try:
        import serial
    except ImportError:
        _fail("缺少 pyserial：请先执行  pip install pyserial numpy pyqtgraph pyqt5")
    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets
    except Exception as e:  # Qt 后端缺失等
        _fail(f"缺少 PyQtGraph/PyQt5（{e}）：请先执行  pip install pyqtgraph pyqt5")

    ser = serial.Serial(port, baud, timeout=1)
    window = max(256, int(fs * window_sec))

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    win = pg.GraphicsLayoutWidget()
    win.setWindowTitle(f"{port} @ {baud} — 波形 / FFT 频谱 / 时频图")
    win.resize(1000, 980)

    plot = win.addPlot(title="INMP441 mono waveform (ch0)")
    plot.setLabel("bottom", "time", units="s")
    plot.setLabel("left", "amplitude")
    plot.setYRange(-1.05, 1.05)
    plot.showGrid(x=True, y=True, alpha=0.3)

    spec_plot = win.addPlot(title="FFT — 人声频段 voice band (ch0)", row=1, col=0)
    spec_plot.setLabel("bottom", "frequency", units="Hz")
    spec_plot.setLabel("left", "magnitude (dBFS)")
    spec_plot.setXRange(VOICE_FMIN, VOICE_FMAX)
    spec_plot.setYRange(-90, 5)
    spec_plot.showGrid(x=True, y=True, alpha=0.3)

    specgram_plot = win.addPlot(title="Spectrogram / STFT — 人声频段 voice band (ch0)", row=2, col=0)
    specgram_plot.setLabel("bottom", "time", units="s")
    specgram_plot.setLabel("left", "frequency", units="Hz")
    specgram_plot.setYRange(VOICE_FMIN, VOICE_FMAX)

    info = win.addLabel("waiting for data...", row=3, col=0)

    parser = FrameParser()
    ch_bufs = {}          # ch -> deque
    lock = threading.Lock()
    running = True

    # 时频图（增量 STFT）状态：spec_inbuf 累积待处理的 ch0 样本，凑满一帧就滑窗算一帧，
    # spec_frames 缓存最近若干帧，避免每次全量重算整个历史
    spec_inbuf = collections.deque()                       # 待处理 ch0 样本流
    n_spec_frames = max(1, int(fs * spec_sec / HOP))       # 时频图保留帧数
    spec_frames = collections.deque(maxlen=n_spec_frames)  # 已算频谱帧，每帧 (freq,)
    spec_win = np.hanning(FFT_N).astype(np.float32)
    spec_total_frames = 0  # 累计计算的频谱帧数，用于时间轴随运行时间递增

    curves = {}
    colors = ["#ffd34d", "#4dff9e", "#4dc3ff", "#ff4d9e"]

    spec_curve = spec_plot.plot(pen=pg.mkPen("#ff4d9e", width=1))
    spec_img = pg.ImageItem()
    specgram_plot.addItem(spec_img)
    cmap = pg.colormap.get("inferno")
    try:
        colorbar = pg.ColorBarItem(values=(-90.0, 0.0), colorMap=cmap, label="dBFS")
        win.addItem(colorbar, row=2, col=1)
        colorbar.setImageItem(spec_img)
    except Exception:
        pass  # 旧版 pyqtgraph 没有 ColorBarItem，仅显示热力图

    def ensure_curve(ch):
        if ch not in curves:
            curves[ch] = plot.plot(pen=pg.mkPen(colors[ch % len(colors)], width=1))
        return curves[ch]

    def reader_loop():
        while running:
            n = ser.in_waiting or 1
            data = ser.read(n)
            if not data:
                continue
            for samples, ch in parser.feed(data):
                for c in range(ch):
                    seg = samples[c::ch]
                    with lock:
                        buf = ch_bufs.setdefault(c, collections.deque(maxlen=window))
                        buf.extend(seg.tolist())
                        if c == 0:
                            spec_inbuf.extend(seg.tolist())

    def update():
        nonlocal spec_total_frames
        with lock:
            arr0 = None
            for ch, buf in ch_bufs.items():
                if not buf:
                    continue
                arr = np.array(list(buf), dtype=np.float32)
                t = (np.arange(arr.size, dtype=np.float64) - (arr.size - 1)) / fs
                ensure_curve(ch).setData(t, arr)
                if ch == 0:
                    arr0 = arr

            # FFT 幅度谱（加窗 + rfft，换算为 dBFS：满幅正弦≈0dB，说话时人声频段会明显凸起）
            if arr0 is not None and arr0.size >= 4:
                x = arr0 - np.float32(np.mean(arr0))
                w = np.hanning(x.size)
                sp = np.abs(np.fft.rfft(x * w))
                ref = x.size / 4.0  # 满幅正弦经 Hann 窗的 rfft 峰值（相干增益 0.5，半谱再除 2）
                freqs = np.fft.rfftfreq(x.size, 1.0 / fs)
                spec_curve.setData(freqs, 20.0 * np.log10(sp / ref + 1e-12))

            # 时频图（增量 STFT）：只对新增样本滑窗算新帧，旧帧复用缓存
            while len(spec_inbuf) >= FFT_N:
                frame = np.array(list(itertools.islice(spec_inbuf, 0, FFT_N)), dtype=np.float32)
                frame -= np.float32(np.mean(frame))
                sp = np.abs(np.fft.rfft(frame * spec_win))
                spec_frames.append(sp)
                spec_total_frames += 1
                for _ in range(HOP):
                    spec_inbuf.popleft()

            if spec_frames:
                s = np.asarray(spec_frames, dtype=np.float32)      # (nframes, freq)
                s_db = 20.0 * np.log10(s / (FFT_N / 4.0) + 1e-12)  # (nframes, freq)
                # col-major 下第 0 维=X（时间）、第 1 维=Y（频率）
                spec_img.setImage(np.ascontiguousarray(s_db), autoLevels=False, levels=(-90.0, 0.0))
                nframes = s_db.shape[0]
                # 时间轴 X：从 0 起随运行递增，最新帧在右；频率轴 Y：0~fs/2（低→高）
                t_newest = ((spec_total_frames - 1) * HOP + FFT_N) / fs
                t_span = (nframes - 1) * HOP / fs if nframes > 1 else FFT_N / fs
                t_oldest = t_newest - t_span
                spec_img.setRect(t_oldest, 0.0, t_span, fs / 2.0)
                specgram_plot.setXRange(t_oldest, t_newest, padding=0)

        info.setText(
            f"frames={parser.frames}  dropped={parser.dropped}  "
            f"buffered={sum(len(b) for b in ch_bufs.values())}  wave={window/fs:.2f}s  "
            f"specgram={len(spec_frames) * HOP / fs:.2f}s"
        )

    thr = threading.Thread(target=reader_loop, daemon=True)
    thr.start()

    # 连接后立刻发一次电脑时间，之后每秒同步一次，显示到 ESP32 的 OLED 屏 0
    def time_sender_loop():
        while running:
            try:
                ser.write(enc_time_frame(datetime.datetime.now()))
            except Exception:
                pass
            time.sleep(1.0)

    threading.Thread(target=time_sender_loop, daemon=True).start()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(33)  # ~30 fps

    win.show()
    try:
        QtWidgets.QApplication.instance().exec_()
    finally:
        running = False
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="实时显示 ESP32 麦克风波形")
    ap.add_argument("port", nargs="?", default=None, help="串口，如 COM5 或 /dev/ttyUSB0（缺省自动枚举）")
    ap.add_argument("baud", nargs="?", type=int, default=921600, help="波特率，默认 921600")
    ap.add_argument("fs", nargs="?", type=int, default=16000, help="采样率 Hz，默认 16000")
    ap.add_argument("--window", type=float, default=0.5, help="波形显示时间窗（秒），默认 0.5")
    ap.add_argument("--spec-window", type=float, default=5.0, help="时频图历史时长（秒），默认 5.0")
    ap.add_argument("--selftest", action="store_true", help="编解码自测（无需硬件）")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    port = args.port or pick_port()
    if not port:
        _fail("未找到可用串口")
    print(f"打开 {port} @ {args.baud}, fs={args.fs} Hz；波形/频谱/时频图窗口弹出后即可查看，关闭窗口即退出")
    run_gui(port, args.baud, args.fs, args.window, args.spec_window)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    except Exception:          # 双击运行时也停留在窗口输出错误，避免闪退
        import traceback
        traceback.print_exc()
        try:
            input("\n出错了，按回车键退出...")
        except Exception:
            pass
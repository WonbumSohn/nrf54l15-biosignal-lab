"""MAX30102 PPG 확인 (MAX30102 PPG check) — Day 2, apps/acq_fw.

PPG(type 1) + PPG_DIAG(0x7B) 프레임을 받아 (Receives PPG frames and diagnostics):
  1) 실행 중: 약 1초마다 Red/IR 평균, IR 맥동 크기(p-p), 손실·오버플로 출력
  2) 끝나면: Part ID, I2C 스캔, 실제 샘플링 레이트(기기 시계), 포화 여부,
     심박수 추정 (IR 0.7–3.5 Hz 대역 피크 + FFT), 손실 요약
  3) 저장: data/raw/ppg_<시각>_<note>/ (session.bin, ppg_raw.csv),
           figures/ppg_<시각>_<note>.png

예 (Example)::

    .venv/bin/python host/ppg_test.py --port /dev/cu.usbmodem03E4434C3 --seconds 30 --note finger
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
import serial  # pyserial
from scipy import signal

sys.path.insert(0, str(Path(__file__).resolve().parent))
from frame import TYPE_PPG, TYPE_PPG_DIAG, FrameParser  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BAUD = 1_000_000  # notes/gate0.md
CH = 2  # red, ir
PPG_DIAG = struct.Struct("<BBBbBBBBB3sIIIIIIHH")  # 40 B, apps/acq_fw/src/max30102.c send_diag()
ADC_MAX = (1 << 18) - 1  # 18비트 (18-bit)
SR_HZ = {0: 50, 1: 100, 2: 200, 3: 400, 4: 800, 5: 1000, 6: 1600, 7: 3200}
ADC_NA = {0: 2048, 1: 4096, 2: 8192, 3: 16384}
PW_US = {0: 69, 1: 118, 2: 215, 3: 411}
HR_BAND_HZ = (0.7, 3.5)  # 42–210 bpm
FINGER_IR_MIN = 10_000  # 이보다 작으면 손가락 없음 (below → no finger; 빈 센서 ≈ 700)


@dataclass
class PpgDiag:
    """PPG 진단 프레임 하나 (One PPG diagnostic frame)."""

    part: int
    rev: int
    init_err: int
    spo2_cfg: int
    fifo_cfg: int
    led_red: int
    led_ir: int
    scan: list[int]
    samples: int
    ovf_samples: int
    i2c_errors: int
    timeouts: int
    tx_drop: int
    reinits: int
    batch_min: int
    batch_max: int

    @classmethod
    def parse(cls, p: bytes) -> "PpgDiag":
        v = PPG_DIAG.unpack_from(p, 0)
        scan_n = v[8]
        return cls(part=v[1], rev=v[2], init_err=v[3], spo2_cfg=v[4], fifo_cfg=v[5],
                   led_red=v[6], led_ir=v[7], scan=list(v[9][:min(scan_n, 3)]),
                   samples=v[10], ovf_samples=v[11], i2c_errors=v[12], timeouts=v[13],
                   tx_drop=v[14], reinits=v[15], batch_min=v[16], batch_max=v[17])

    @property
    def rate_hz(self) -> int:
        return SR_HZ[(self.spo2_cfg >> 2) & 7]

    def describe(self) -> str:
        """설정을 사람이 읽을 수 있게 (human-readable config)."""
        return (f"{self.rate_hz} Hz, 펄스폭 {PW_US[self.spo2_cfg & 3]} µs, "
                f"ADC {ADC_NA[(self.spo2_cfg >> 5) & 3]} nA, "
                f"LED Red {self.led_red * 0.2:.1f} mA / IR {self.led_ir * 0.2:.1f} mA, "
                f"A_FULL {32 - (self.fifo_cfg & 0xF)}개")


@dataclass
class PpgCollector:
    """PPG 샘플을 first_idx로 이어 붙임 (Stitch PPG samples by first_idx)."""

    chunks: list[np.ndarray] = field(default_factory=list)
    t_dev: list[tuple[int, int]] = field(default_factory=list)  # (last idx, t_dev_us)
    first: int | None = None
    next_idx: int | None = None
    missing: int = 0
    frames: int = 0

    def add(self, first_idx: int, t_dev_us: int, block: np.ndarray) -> None:
        if self.first is None:
            self.first = first_idx
        if self.next_idx is not None and first_idx != self.next_idx:
            gap = (first_idx - self.next_idx) & 0xFFFFFFFF
            if gap < 100_000:  # 손실 → NaN (lost samples → NaN)
                self.chunks.append(np.full((gap, CH), np.nan))
                self.missing += gap
        self.chunks.append(block.astype(float))
        self.next_idx = (first_idx + len(block)) & 0xFFFFFFFF
        self.t_dev.append((first_idx + len(block) - 1, t_dev_us))
        self.frames += 1

    def samples(self) -> np.ndarray:
        return np.concatenate(self.chunks) if self.chunks else np.zeros((0, CH))


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="시리얼 포트 (serial port)")
    ap.add_argument("--replay", type=Path,
                    help="저장된 session.bin을 다시 분석 (re-analyse a saved session.bin)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--note", default="", help="파일 이름에 붙일 메모 (e.g. finger, nofinger)")
    ap.add_argument("--no-save", action="store_true", help="파일 저장 안 함")
    return ap.parse_args()


def receive(port: str, seconds: float
            ) -> tuple[PpgCollector, list[PpgDiag], bytearray, FrameParser]:
    """seconds 동안 수신하며 1초마다 상태 출력 (Receive and print live stats)."""
    parser = FrameParser()
    col = PpgCollector()
    diags: list[PpgDiag] = []
    raw = bytearray()
    win: list[np.ndarray] = []

    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit(f"포트를 열 수 없음 (cannot open port): {exc}")

    print(f"{seconds:.0f}초 수신 시작 (receiving)…  "
          "[시간, Red 평균, IR 평균, IR p-p, 프레임, 손실, FIFO 넘침]")
    with ser:
        ser.reset_input_buffer()
        t0 = time.monotonic()
        t_print = t0
        while (now := time.monotonic()) - t0 < seconds:
            try:
                chunk = ser.read(max(1, ser.in_waiting))
            except serial.SerialException as exc:
                print(f"\n수신 오류 (read error): {exc}")
                break
            if not chunk:
                continue
            raw += chunk
            for fr in parser.feed(chunk):
                if fr.type == TYPE_PPG and len(fr.payload) % (4 * CH) == 0:
                    block = np.frombuffer(fr.payload, dtype="<i4").reshape(-1, CH)
                    col.add(fr.first_idx, fr.t_dev_us, block)
                    win.append(block)
                elif fr.type == TYPE_PPG_DIAG and len(fr.payload) >= PPG_DIAG.size:
                    d = PpgDiag.parse(fr.payload)
                    diags.append(d)
                    if d.init_err:
                        print(f"⚠️ MAX30102 초기화 실패 (init failed): err {d.init_err}, "
                              f"Part ID 0x{d.part:02X}, I2C 스캔 "
                              f"{[hex(a) for a in d.scan] or '응답 없음 (none)'}")
            if now - t_print >= 1.0 and win:
                x = np.concatenate(win).astype(float)
                ovf = diags[-1].ovf_samples if diags else 0
                print(f"{now - t0:5.1f}s  Red {x[:, 0].mean():8.0f}  IR {x[:, 1].mean():8.0f}  "
                      f"IR p-p {np.ptp(x[:, 1]):6.0f}  프레임 {col.frames:4d}  "
                      f"손실 {col.missing:3d}  넘침 {ovf}")
                win.clear()
                t_print = now
    return col, diags, raw, parser


def replay(path: Path) -> tuple[PpgCollector, list[PpgDiag], bytearray, FrameParser]:
    """저장된 session.bin을 같은 방식으로 파싱 (parse a saved session.bin)."""
    parser = FrameParser()
    col = PpgCollector()
    diags: list[PpgDiag] = []
    raw = bytearray(path.read_bytes())
    for fr in parser.feed(bytes(raw)):
        if fr.type == TYPE_PPG and len(fr.payload) % (4 * CH) == 0:
            col.add(fr.first_idx, fr.t_dev_us,
                    np.frombuffer(fr.payload, dtype="<i4").reshape(-1, CH))
        elif fr.type == TYPE_PPG_DIAG and len(fr.payload) >= PPG_DIAG.size:
            diags.append(PpgDiag.parse(fr.payload))
    return col, diags, raw, parser


def device_rate(col: PpgCollector) -> float:
    """기기 시계로 잰 실제 샘플링 레이트 [Hz] (actual rate from device timestamps)."""
    if len(col.t_dev) < 2:
        return float("nan")
    idx = np.array([i for i, _ in col.t_dev], dtype=float)
    t = np.unwrap(np.array([t for _, t in col.t_dev], dtype=float), period=2 ** 32) / 1e6
    return 1.0 / np.polyfit(idx, t, 1)[0]


def pulse_filter(x: np.ndarray, fs: float) -> np.ndarray:
    """맥파 대역 통과 (pulse band-pass, 0.7–3.5 Hz). NaN은 선형 보간 후 처리."""
    y = x.copy()
    bad = np.isnan(y)
    if bad.all():
        return y
    if bad.any():
        y[bad] = np.interp(np.flatnonzero(bad), np.flatnonzero(~bad), y[~bad])
    sos = signal.butter(2, HR_BAND_HZ, btype="bandpass", fs=fs, output="sos")
    return signal.sosfiltfilt(sos, y)


def heart_rate(ac: np.ndarray, fs: float) -> tuple[float, float, np.ndarray]:
    """심박수 [bpm]: 피크 간격 중앙값, FFT 피크 (peak-interval and FFT estimates)."""
    skip = int(2 * fs)  # 필터 가장자리 2초 제외 (skip filter edges)
    seg = ac[skip:-skip] if len(ac) > 4 * skip else ac
    if len(seg) < 4 * fs:
        return float("nan"), float("nan"), np.array([], dtype=int)
    peaks, _ = signal.find_peaks(seg, distance=int(fs / HR_BAND_HZ[1]),
                                 prominence=0.3 * np.std(seg))
    hr_peak = 60.0 * fs / np.median(np.diff(peaks)) if len(peaks) >= 3 else float("nan")
    f, pxx = signal.welch(seg, fs=fs, nperseg=min(len(seg), int(16 * fs)))
    band = (f >= HR_BAND_HZ[0]) & (f <= HR_BAND_HZ[1])
    hr_fft = 60.0 * f[band][np.argmax(pxx[band])] if band.any() else float("nan")
    return hr_peak, hr_fft, peaks + (skip if len(ac) > 4 * skip else 0)


def finger_segment(ir: np.ndarray, fs: float, trim_s: float = 1.5) -> slice | None:
    """손가락이 올라가 있는 가장 긴 구간, 앞뒤 trim_s 초 제외 (longest finger-on run).

    올리고 뗄 때의 큰 계단은 대역 통과 후에도 심박수 계산을 망치므로 제외함.
    The on/off steps would dominate the band-passed signal, so they are cut off.
    """
    on = np.nan_to_num(ir) >= FINGER_IR_MIN
    best, start = (0, 0), None
    for i, v in enumerate(np.append(on, False)):
        if v and start is None:
            start = i
        elif not v and start is not None:
            if i - start > best[1] - best[0]:
                best = (start, i)
            start = None
    trim = int(trim_s * fs)
    a, b = best[0] + trim, best[1] - trim
    return slice(a, b) if b - a >= int(5 * fs) else None


def report(col: PpgCollector, diags: list[PpgDiag], parser: FrameParser
           ) -> tuple[np.ndarray, float]:
    """칩·레이트·신호 판정 보고 (Report chip, rate and signal checks)."""
    d = diags[-1]
    x = col.samples()
    fs = device_rate(col)

    print("\n=== 칩·설정 (Chip and config) ===")
    print(f"Part ID 0x{d.part:02X} (기대 0x15), Rev {d.rev}, "
          f"I2C 스캔 {[hex(a) for a in d.scan]}")
    print(f"설정: {d.describe()}")
    print(f"실제 레이트 (기기 시계, device clock) = {fs:.3f} Hz "
          f"(설정 대비 {100 * (fs / d.rate_hz - 1):+.2f}%)")
    print(f"한 번에 읽은 샘플 수 (마지막 창) {d.batch_min}–{d.batch_max}")

    valid = ~np.isnan(x).any(axis=1)
    if valid.any():
        red, ir = x[valid, 0], x[valid, 1]
        print("\n=== 신호 (Signal) ===")
        for name, v in (("Red", red), ("IR", ir)):
            sat = np.mean(v >= ADC_MAX - 16) * 100
            print(f"{name:3s}: 평균 {v.mean():8.0f}  최소 {v.min():8.0f}  최대 {v.max():8.0f}  "
                  f"({100 * v.mean() / ADC_MAX:4.1f}% of full scale, 포화 {sat:.1f}%)")
        seg = finger_segment(x[:, 1], fs)
        if seg is None:
            print(f"⚠️ IR ≥ {FINGER_IR_MIN} 인 구간이 5초 미만 → 손가락 없음, 심박수 계산 생략 "
                  "(no finger detected)")
        else:
            xs = x[seg]
            ac = pulse_filter(xs[:, 1], fs)
            hr_peak, hr_fft, peaks = heart_rate(ac, fs)
            dc = np.nanmean(xs[:, 1])
            pp = np.percentile(ac, 95) - np.percentile(ac, 5)
            print(f"손가락 구간 (finger-on) {seg.start / fs:.1f}–{seg.stop / fs:.1f} s: "
                  f"Red 평균 {np.nanmean(xs[:, 0]):.0f}, IR 평균 {dc:.0f} "
                  f"({100 * dc / ADC_MAX:.1f}% of full scale)")
            print(f"IR 맥동 p-p {pp:.0f} counts, AC/DC = {100 * pp / dc:.2f}% "
                  "(관류 지수 근사, perfusion index)")
            print(f"심박수 (heart rate): 피크 간격 {hr_peak:.1f} bpm ({len(peaks)}개 피크), "
                  f"FFT {hr_fft:.1f} bpm")

    print("\n=== 요약 (Summary) ===")
    print(f"샘플 {len(x)}개 = {len(x) / fs:.2f} s, 프레임 {col.frames}, 손실 샘플 {col.missing}, "
          f"FIFO 넘침 {d.ovf_samples}, CRC 오류 {parser.crc_errors}, I2C 오류 {d.i2c_errors}, "
          f"INT 시간 초과 {d.timeouts}, 칩 재초기화 {d.reinits}, 장치 송신 버림 {d.tx_drop}")
    return x, fs


def save(out: Path, raw: bytearray, col: PpgCollector, x: np.ndarray, fs: float) -> None:
    """원본 bin과 ppg_raw.csv(sample_idx, t_s, red, ir) 저장 (save raw bin and CSV)."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "session.bin").write_bytes(bytes(raw))
    first = col.first or 0
    with (out / "ppg_raw.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sample_idx", "t_s", "red", "ir"])
        for i, (r, ir) in enumerate(x):
            if np.isnan(r):
                continue
            w.writerow([first + i, f"{i / fs:.4f}", int(r), int(ir)])


def plot(x: np.ndarray, fs: float, path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(len(x)) / fs
    seg = finger_segment(x[:, 1], fs) or slice(0, len(x))
    ts = t[seg]
    ac = pulse_filter(x[seg, 1], fs)
    _, _, peaks = heart_rate(ac, fs)
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 7.5))
    a1.plot(t, x[:, 0], lw=0.8, color="tab:red", label="Red")
    a1.plot(t, x[:, 1], lw=0.8, color="tab:purple", label="IR")
    a1.set(ylabel="Raw [ADC counts]", title=title)
    a1.legend(loc="upper right")
    a1.axvspan(ts[0], ts[-1], color="tab:green", alpha=0.08, label="analysed")
    a2.plot(ts, x[seg, 1], lw=0.8, color="tab:purple")
    a2.set(ylabel="IR raw (finger-on)")
    a3.plot(ts, ac, lw=0.8, color="k")
    if len(peaks):
        a3.plot(ts[peaks], ac[peaks], "o", ms=4, color="tab:orange", label="peaks")
        a3.legend(loc="upper right")
    a3.set(xlabel="Time [s]", ylabel="IR band-pass 0.7–3.5 Hz")
    for ax in (a1, a2, a3):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.replay:
        col, diags, raw, parser = replay(args.replay)
    elif args.port:
        col, diags, raw, parser = receive(args.port, args.seconds)
    else:
        sys.exit("--port 또는 --replay 가 필요함 (need --port or --replay)")

    if not diags:
        sys.exit("PPG 진단 프레임 0개 — acq_fw(PPG=1) 플래시·포트 확인 (no PPG diagnostic frames)")
    if diags[-1].init_err or not col.frames:
        d = diags[-1]
        sys.exit(f"PPG 데이터 없음 (no PPG data): Part ID 0x{d.part:02X}, init_err {d.init_err}, "
                 f"I2C 스캔 {[hex(a) for a in d.scan]}")

    x, fs = report(col, diags, parser)
    if args.replay:  # 원본 폴더 이름으로 그림만 다시 그림 (redraw the figure only)
        tag = args.replay.resolve().parent.name
        fig = ROOT / "figures" / f"{tag}.png"
        plot(x, fs, fig, tag)
        print(f"\n그림 (figure): {fig}")
        return
    if args.no_save:
        return
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"ppg_{stamp}" + (f"_{args.note}" if args.note else "")
    out = ROOT / "data" / "raw" / tag
    save(out, raw, col, x, fs)
    fig = ROOT / "figures" / f"{tag}.png"
    plot(x, fs, fig, tag)
    print(f"\n저장 (saved): {out}/ (session.bin, ppg_raw.csv)\n그림 (figure): {fig}")


if __name__ == "__main__":
    main()

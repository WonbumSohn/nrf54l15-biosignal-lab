"""내장 IMU 확인 (On-board IMU check) — XIAO nRF54L15 Sense, apps/acq_fw.

IMU(type 6) + IMU_DIAG(0x7C) 프레임을 받아 (Receives IMU frames and diagnostics):
  1) 실행 중: 약 1초마다 가속도 평균[g]·|a|·자이로 평균[dps]·온도·손실 출력
  2) 끝나면: 칩 종류(WHO_AM_I), 실제 ODR(기기 시계 기준), 정지 상태 판정
     (|a| ≈ 1 g, 자이로 영점), 잡음, 손실 요약
  3) 저장: data/raw/imu_<시각>_<note>/ (session.bin, imu.csv),
           figures/imu_<시각>_<note>.png

예 (Example)::

    .venv/bin/python host/imu_test.py --port /dev/cu.usbmodem03E4434C3 --seconds 10 --note still
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from frame import TYPE_IMU, TYPE_IMU_DIAG, FrameParser  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BAUD = 1_000_000  # notes/gate0.md
AXES = 6  # ax, ay, az, gx, gy, gz
IMU_DIAG = struct.Struct("<BBbBHHHhIIIIIII")  # 40 B, apps/acq_fw/src/imu.c send_diag()
CHIPS = {0x6A: "LSM6DS3TR-C", 0x6C: "LSM6DSO"}


@dataclass
class ImuDiag:
    """IMU 진단 프레임 하나 (One IMU diagnostic frame)."""

    who: int
    init_err: int
    odr_hz: int
    fs_g: int
    fs_dps: int
    temp_c: float
    samples: int
    gaps: int
    i2c_errors: int
    timeouts: int
    tx_drop: int
    dt_min_us: int
    dt_max_us: int

    @classmethod
    def parse(cls, p: bytes) -> "ImuDiag":
        v = IMU_DIAG.unpack_from(p, 0)
        return cls(who=v[1], init_err=v[2], odr_hz=v[4], fs_g=v[5], fs_dps=v[6],
                   temp_c=25.0 + v[7] / 256.0, samples=v[8], gaps=v[9], i2c_errors=v[10],
                   timeouts=v[11], tx_drop=v[12], dt_min_us=v[13], dt_max_us=v[14])

    @property
    def chip(self) -> str:
        return CHIPS.get(self.who, f"알 수 없음 (unknown) 0x{self.who:02X}")

    @property
    def g_per_lsb(self) -> float:
        """가속도 감도 [g/LSB] (±2/4/8/16 g → 0.061/0.122/0.244/0.488 mg/LSB)."""
        return 0.061e-3 * (self.fs_g / 2)

    @property
    def dps_per_lsb(self) -> float:
        """자이로 감도 [dps/LSB] (±250/500/1000/2000 → 8.75/17.5/35/70 mdps/LSB)."""
        return 8.75e-3 * (self.fs_dps / 250)


@dataclass
class ImuCollector:
    """IMU 샘플을 first_idx로 이어 붙임 (Stitch IMU samples by first_idx)."""

    chunks: list[np.ndarray] = field(default_factory=list)
    t_dev: list[tuple[int, int]] = field(default_factory=list)  # (last idx, t_dev_us)
    next_idx: int | None = None
    missing: int = 0
    frames: int = 0

    def add(self, first_idx: int, t_dev_us: int, block: np.ndarray) -> None:
        if self.next_idx is not None and first_idx != self.next_idx:
            gap = (first_idx - self.next_idx) & 0xFFFFFFFF
            if gap < 100_000:  # 손실 → NaN (lost samples → NaN)
                self.chunks.append(np.full((gap, AXES), np.nan))
                self.missing += gap
        self.chunks.append(block.astype(float))
        self.next_idx = (first_idx + len(block)) & 0xFFFFFFFF
        self.t_dev.append((first_idx + len(block) - 1, t_dev_us))
        self.frames += 1

    def samples(self) -> np.ndarray:
        return np.concatenate(self.chunks) if self.chunks else np.zeros((0, AXES))


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--note", default="", help="파일 이름에 붙일 메모 (e.g. still, tilt, shake)")
    ap.add_argument("--no-save", action="store_true", help="파일 저장 안 함")
    return ap.parse_args()


def receive(port: str, seconds: float
            ) -> tuple[ImuCollector, list[ImuDiag], bytearray, FrameParser]:
    """seconds 동안 수신하며 1초마다 상태 출력 (Receive and print live stats)."""
    parser = FrameParser()
    col = ImuCollector()
    diags: list[ImuDiag] = []
    raw = bytearray()
    win: list[np.ndarray] = []

    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit(f"포트를 열 수 없음 (cannot open port): {exc}")

    print(f"{seconds:.0f}초 수신 시작 (receiving)…  "
          "[시간, 가속도 평균 x/y/z [g], |a|, 자이로 평균 x/y/z [dps], 온도, 프레임, 손실]")
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
                if fr.type == TYPE_IMU and len(fr.payload) % (2 * AXES) == 0:
                    block = np.frombuffer(fr.payload, dtype="<i2").reshape(-1, AXES)
                    col.add(fr.first_idx, fr.t_dev_us, block)
                    win.append(block)
                elif fr.type == TYPE_IMU_DIAG and len(fr.payload) >= IMU_DIAG.size:
                    d = ImuDiag.parse(fr.payload)
                    diags.append(d)
                    if d.init_err:
                        print(f"⚠️ IMU 초기화 실패 (init failed): err {d.init_err}, "
                              f"WHO_AM_I 0x{d.who:02X} ({d.chip})")
            if now - t_print >= 1.0 and win and diags:
                d = diags[-1]
                x = np.concatenate(win).astype(float)
                acc = x[:, :3] * d.g_per_lsb
                gyr = x[:, 3:] * d.dps_per_lsb
                a = acc.mean(axis=0)
                g = gyr.mean(axis=0)
                print(f"{now - t0:5.1f}s  a {a[0]:+6.3f} {a[1]:+6.3f} {a[2]:+6.3f}  "
                      f"|a| {np.linalg.norm(acc, axis=1).mean():5.3f}  "
                      f"g {g[0]:+7.2f} {g[1]:+7.2f} {g[2]:+7.2f}  {d.temp_c:5.1f}°C  "
                      f"프레임 {col.frames:4d}  손실 {col.missing:4d}  crc_err {parser.crc_errors}")
                win.clear()
                t_print = now
    return col, diags, raw, parser


def device_rate(col: ImuCollector) -> float:
    """기기 시계로 잰 실제 샘플링 레이트 [Hz] (actual ODR from device timestamps)."""
    if len(col.t_dev) < 2:
        return float("nan")
    idx = np.array([i for i, _ in col.t_dev], dtype=float)
    t = np.unwrap(np.array([t for _, t in col.t_dev], dtype=float), period=2 ** 32) / 1e6
    slope = np.polyfit(idx, t, 1)[0]  # 샘플당 초 (seconds per sample)
    return 1.0 / slope


def report(col: ImuCollector, diags: list[ImuDiag], parser: FrameParser
           ) -> tuple[np.ndarray, np.ndarray, float]:
    """칩·ODR·정지 상태 판정 보고 (Report chip, ODR and at-rest checks)."""
    d = diags[-1]
    x = col.samples()
    acc = x[:, :3] * d.g_per_lsb
    gyr = x[:, 3:] * d.dps_per_lsb
    fs = device_rate(col)

    print("\n=== 칩·설정 (Chip and config) ===")
    print(f"WHO_AM_I 0x{d.who:02X} → {d.chip}, 설정 ODR {d.odr_hz} Hz, "
          f"±{d.fs_g} g, ±{d.fs_dps} dps, 온도 {d.temp_c:.1f} °C")
    print(f"실제 ODR (기기 시계, device clock) = {fs:.2f} Hz "
          f"(설정 대비 {100 * (fs / d.odr_hz - 1):+.2f}%)")
    print(f"샘플 간격 (마지막 1초 창 / last window) {d.dt_min_us}–{d.dt_max_us} µs, "
          f"간격 > 1.5 주기 누적 {d.gaps}회")

    valid = ~np.isnan(acc).any(axis=1)
    if valid.any():
        mag = np.linalg.norm(acc[valid], axis=1)
        print("\n=== 정지 상태 확인 (At-rest check, 보드를 움직이지 않았을 때 의미 있음) ===")
        print(f"|a| 평균 {mag.mean():.4f} g (기대 1.000), 표준편차 {mag.std() * 1e3:.2f} mg")
        print("가속도 평균 x/y/z [g]: " + " ".join(f"{v:+.4f}" for v in acc[valid].mean(axis=0)))
        print("자이로 평균 x/y/z [dps] (영점 / bias): "
              + " ".join(f"{v:+.3f}" for v in gyr[valid].mean(axis=0)))
        print("자이로 표준편차 x/y/z [dps] (잡음 / noise): "
              + " ".join(f"{v:.3f}" for v in gyr[valid].std(axis=0)))

    print("\n=== 요약 (Summary) ===")
    print(f"샘플 {len(x)}개 = {len(x) / fs:.2f} s, 프레임 {col.frames}, 손실 샘플 {col.missing}, "
          f"CRC 오류 {parser.crc_errors}, I2C 오류 {d.i2c_errors}, INT 시간 초과 {d.timeouts}, "
          f"장치 송신 버림 {d.tx_drop}")
    return acc, gyr, fs


def save(out: Path, raw: bytearray, acc: np.ndarray, gyr: np.ndarray, fs: float) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "session.bin").write_bytes(bytes(raw))
    with (out / "imu.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"])
        for i, (a, g) in enumerate(zip(acc, gyr)):
            w.writerow([f"{i / fs:.5f}", *(f"{v:.5f}" for v in a), *(f"{v:.4f}" for v in g)])


def plot(acc: np.ndarray, gyr: np.ndarray, fs: float, path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(len(acc)) / fs
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for k, name in enumerate("xyz"):
        a1.plot(t, acc[:, k], lw=0.8, label=f"a{name}")
        a3.plot(t, gyr[:, k], lw=0.8, label=f"g{name}")
    a2.plot(t, np.linalg.norm(acc, axis=1), lw=0.8, color="k")
    a1.set(ylabel="Accel [g]", title=title)
    a2.set(ylabel="|a| [g]")
    a3.set(xlabel="Time [s]", ylabel="Gyro [dps]")
    for ax in (a1, a2, a3):
        ax.grid(alpha=0.3)
    a1.legend(loc="upper right", ncol=3)
    a3.legend(loc="upper right", ncol=3)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    col, diags, raw, parser = receive(args.port, args.seconds)

    if not diags:
        sys.exit("IMU 진단 프레임 0개 — acq_fw(IMU=1) 플래시·포트 확인 (no IMU diagnostic frames)")
    if diags[-1].init_err or not col.frames:
        sys.exit(f"IMU 데이터 없음 (no IMU data): WHO_AM_I 0x{diags[-1].who:02X}, "
                 f"init_err {diags[-1].init_err}")

    acc, gyr, fs = report(col, diags, parser)
    if args.no_save:
        return
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"imu_{stamp}" + (f"_{args.note}" if args.note else "")
    out = ROOT / "data" / "raw" / tag
    save(out, raw, acc, gyr, fs)
    fig = ROOT / "figures" / f"{tag}.png"
    plot(acc, gyr, fs, fig, tag)
    print(f"\n저장 (saved): {out}/ (session.bin, imu.csv)\n그림 (figure): {fig}")


if __name__ == "__main__":
    main()

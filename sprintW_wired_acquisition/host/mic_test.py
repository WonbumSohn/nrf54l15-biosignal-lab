"""Day 3 — 마이크 확인: SPH0645(I2S) 또는 내장 PDM 마이크 (Microphone check).

apps/acq_fw 가 보내는 프레임을 받아 (Receives frames from apps/acq_fw):
  --source i2s : MIC(type 2) + MIC_DIAG(0x7E)       — SPH0645
  --source pdm : MIC_PDM(type 5) + PDM_DIAG(0x7D)   — XIAO 내장 마이크 (on-board)

  1) 실행 중: 약 1초마다 값 범위·평균(DC)·손실 출력 (live stats)
  2) 끝나면: 실제 클럭(레지스터 기준), 채널·정렬 판정, 손실 요약 (report)
  3) 저장: data/raw/mic_<source>_<시각>/ 에 session.bin, mic.wav, mic.csv
           figures/mic_<source>_<시각>.png (파형·스펙트럼 / waveform + spectrum)

예 (Example)::

    .venv/bin/python host/mic_test.py --port /dev/cu.usbmodem03E4434C3 --source pdm --note quiet
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
import wave
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
import serial  # pyserial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from frame import (TYPE_MIC, TYPE_MIC_DIAG, TYPE_MIC_PDM, TYPE_PDM_DIAG,  # noqa: E402
                   FrameParser)

ROOT = Path(__file__).resolve().parents[1]
BAUD = 1_000_000  # notes/gate0.md 결정 (decision)
PCLK = 32_000_000
I2S_RATIOS = [32, 48, 64, 96, 128, 192, 256, 384, 512]  # I2S CONFIG.RATIO enum → 값
PDM_RATIOS = [32, 48, 50, 64, 80, 96, 100, 128]         # PDM RATIO enum → 값 (nrf54l15_types.h)
I2S_DIAG = struct.Struct("<BBHiiiIIII8IIIII")  # 80 B, apps/acq_fw/src/mic.c send_diag()
PDM_DIAG = struct.Struct("<BBHiiiiiiIIIII")    # 48 B, apps/acq_fw/src/mic_pdm.c send_diag()
I2S_DECIM = 4
PDM_DECIM = 2


@dataclass
class Diag:
    """진단 프레임 하나 — 두 경로 공통 형태 (One diagnostic frame, common view).

    I2S: L 값은 18비트, R은 원본 워드 OR/AND만 의미 있음.
    PDM: L·R 모두 int16 통계 (both channels as int16 stats).
    """

    raw_mode: bool       # I2S MIC_RAW=1 빌드 여부 (PDM은 항상 False)
    n: int
    l_min: int
    l_max: int
    l_mean: int
    r_min: int = 0
    r_max: int = 0
    r_mean: int = 0
    l_or: int = 0
    l_and: int = 0xFFFFFFFF
    r_or: int = 0
    r_and: int = 0xFFFFFFFF
    words: tuple[int, ...] = ()
    errors: int = 0
    tx_drop: int = 0
    out_ch: int = 0
    regs: tuple[int, ...] = ()  # I2S: (MCKFREQ, RATIO) / PDM: (PRESCALER, RATIO, GAINL)

    @classmethod
    def parse_i2s(cls, p: bytes) -> "Diag":
        v = I2S_DIAG.unpack_from(p, 0)
        return cls(raw_mode=v[1] == 1, n=v[2], l_min=v[3], l_max=v[4], l_mean=v[5],
                   l_or=v[6], l_and=v[7], r_or=v[8], r_and=v[9], words=tuple(v[10:18]),
                   errors=v[18], tx_drop=v[19], regs=(v[20], v[21]))

    @classmethod
    def parse_pdm(cls, p: bytes) -> "Diag":
        v = PDM_DIAG.unpack_from(p, 0)
        return cls(raw_mode=False, out_ch=v[1], n=v[2], l_min=v[3], l_max=v[4], l_mean=v[5],
                   r_min=v[6], r_max=v[7], r_mean=v[8], errors=v[9], tx_drop=v[10],
                   regs=(v[11], v[12], v[13]))


@dataclass(frozen=True)
class Source:
    """마이크 경로별 설정 (Per-path settings)."""

    name: str
    mic_type: int
    diag_type: int
    diag_size: int
    parse: object  # Callable[[bytes], Diag]


SOURCES = {
    "i2s": Source("i2s", TYPE_MIC, TYPE_MIC_DIAG, I2S_DIAG.size, Diag.parse_i2s),
    "pdm": Source("pdm", TYPE_MIC_PDM, TYPE_PDM_DIAG, PDM_DIAG.size, Diag.parse_pdm),
}


@dataclass
class Collector:
    """MIC 샘플을 first_idx로 이어 붙임 (Stitch MIC samples by first_idx)."""

    chunks: list[np.ndarray] = field(default_factory=list)
    next_idx: int | None = None
    first_idx0: int | None = None
    missing: int = 0
    frames: int = 0

    def add(self, first_idx: int, samples: np.ndarray) -> None:
        if self.next_idx is None:
            self.first_idx0 = first_idx
        elif first_idx != self.next_idx:
            gap = (first_idx - self.next_idx) & 0xFFFFFFFF
            if gap < 1_000_000:  # 손실 → 0으로 채움 (lost → zero-fill)
                self.chunks.append(np.zeros(gap, dtype=np.int16))
                self.missing += gap
        self.chunks.append(samples)
        self.next_idx = (first_idx + len(samples)) & 0xFFFFFFFF
        self.frames += 1

    def samples(self) -> np.ndarray:
        return np.concatenate(self.chunks) if self.chunks else np.zeros(0, np.int16)


def bits(x: int) -> str:
    """32비트를 8비트씩 끊어 표시 (32-bit value as grouped binary)."""
    s = f"{x & 0xFFFFFFFF:032b}"
    return " ".join(s[i:i + 8] for i in range(0, 32, 8))


def i2s_clocks(d: Diag) -> tuple[float, float]:
    """(MCK, LRCK) [Hz] — PS 8.11.5: MCK = 32 MHz / (2^20 // (MCKFREQ/4096))."""
    mckfreq, ratio_reg = d.regs
    div = (1 << 20) // max(1, mckfreq // 4096)
    mck = PCLK / div
    ratio = I2S_RATIOS[ratio_reg] if ratio_reg < len(I2S_RATIOS) else 0
    return mck, (mck / ratio if ratio else 0.0)


def pdm_clocks(d: Diag) -> tuple[float, float]:
    """(PDM CLK, 캡처 fs) [Hz] — CLK = 32 MHz / PRESCALER, fs = CLK / RATIO."""
    prescaler, ratio_reg, _ = d.regs
    clk = PCLK / max(1, prescaler)
    ratio = PDM_RATIOS[ratio_reg] if ratio_reg < len(PDM_RATIOS) else 0
    return clk, (clk / ratio if ratio else 0.0)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--source", choices=sorted(SOURCES), default="pdm",
                    help="pdm = 내장 마이크 (on-board), i2s = SPH0645")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--note", default="", help="파일 이름에 붙일 메모 (e.g. quiet, clap, speech)")
    ap.add_argument("--no-save", action="store_true", help="파일 저장 안 함")
    return ap.parse_args()


def receive(port: str, seconds: float, src: Source
            ) -> tuple[Collector, list[Diag], bytearray, FrameParser]:
    """seconds 동안 수신하며 1초마다 상태 출력 (Receive and print live stats)."""
    parser = FrameParser()
    col = Collector()
    diags: list[Diag] = []
    raw = bytearray()
    win: list[Diag] = []

    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit(f"포트를 열 수 없음 (cannot open port): {exc}")

    print(f"[{src.name}] {seconds:.0f}초 수신 시작 (receiving)…  "
          "[시간, L 최소/최대/평균, R 최소/최대/평균, 프레임, 손실]")
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
                if fr.type == src.mic_type:
                    col.add(fr.first_idx, np.frombuffer(fr.payload, dtype="<i2").copy())
                elif fr.type == src.diag_type and len(fr.payload) >= src.diag_size:
                    d = src.parse(fr.payload)
                    diags.append(d)
                    win.append(d)
            if now - t_print >= 1.0 and win:
                r_part = (f"R {min(d.r_min for d in win):7d} / {max(d.r_max for d in win):7d} / "
                          f"{int(np.mean([d.r_mean for d in win])):7d}   "
                          if src.name == "pdm" else "")
                print(f"{now - t0:5.1f}s  L {min(d.l_min for d in win):7d} / "
                      f"{max(d.l_max for d in win):7d} / {int(np.mean([d.l_mean for d in win])):7d}"
                      f"   {r_part}프레임 {col.frames:5d}  손실 {col.missing:6d}  "
                      f"crc_err {parser.crc_errors}")
                win.clear()
                t_print = now
    return col, diags, raw, parser


def report_i2s(diags: list[Diag]) -> float:
    """I2S 클럭·비트 정렬 보고, 출력 fs 반환 (Report I2S clocks/alignment; return fs)."""
    last = diags[-1]
    mck, lrck = i2s_clocks(last)
    fs = lrck if last.raw_mode else lrck / I2S_DECIM
    or_l = or_r = 0
    and_l = and_r = 0xFFFFFFFF
    for d in diags:
        or_l |= d.l_or
        and_l &= d.l_and
        or_r |= d.r_or
        and_r &= d.r_and

    print("\n=== I2S 클럭 (드라이버가 쓴 레지스터 기준 / from registers) ===")
    print(f"MCK {mck / 1e6:.4f} MHz, LRCK {lrck:.2f} Hz, SCK {lrck * 64 / 1e6:.4f} MHz")
    print(f"출력 샘플링 레이트 (output fs) = {fs:.3f} Hz  "
          f"({'원본 raw' if last.raw_mode else '÷4 decimated'})")
    print("\n=== 비트 마스크 (전체 기간 OR / AND) — 정렬 확인 (alignment check) ===")
    print(f"L OR : {bits(or_l)}\nL AND: {bits(and_l)}\nR OR : {bits(or_r)}\nR AND: {bits(and_r)}")
    low14 = or_l & 0x3FFF
    print(f"→ L 하위 14비트 사용: {'없음 ✅ (>>14 정렬 OK)' if low14 == 0 else f'있음 ⚠️ 0x{low14:04X}'}")
    print(f"→ R 채널: {'0 ✅' if or_r == 0 else '값 있음 ⚠️ (SEL·채널·풀다운 확인)'}")
    print("최근 원본 워드 (last raw words L,R,…): " + " ".join(f"{w:08X}" for w in last.words))
    return fs


def report_pdm(diags: list[Diag]) -> float:
    """PDM 클럭·채널 보고, 출력 fs 반환 (Report PDM clocks/channels; return fs)."""
    last = diags[-1]
    clk, fs_cap = pdm_clocks(last)
    fs = fs_cap / PDM_DECIM
    print("\n=== PDM 클럭 (드라이버가 쓴 레지스터 기준 / from registers) ===")
    print(f"PRESCALER={last.regs[0]} RATIO={PDM_RATIOS[last.regs[1]] if last.regs[1] < 8 else '?'}"
          f" GAINL=0x{last.regs[2]:02X}  → PDM CLK {clk / 1e6:.4f} MHz, 캡처 {fs_cap:.2f} Hz")
    print(f"출력 샘플링 레이트 (output fs) = {fs:.3f} Hz  (÷{PDM_DECIM} decimated)")

    print("\n=== 채널별 범위 (전체 기간) — 마이크가 어느 쪽인지 (which edge has the mic) ===")
    spans = []
    for name, lo, hi, mean in (
        ("L", min(d.l_min for d in diags), max(d.l_max for d in diags),
         np.mean([d.l_mean for d in diags])),
        ("R", min(d.r_min for d in diags), max(d.r_max for d in diags),
         np.mean([d.r_mean for d in diags])),
    ):
        spans.append(hi - lo)
        print(f"{name}: 최소 {lo:7d}, 최대 {hi:7d}, 평균 {mean:9.1f}, 폭 {hi - lo:6d}")
    out = "LR"[last.out_ch]
    print(f"→ 출력 채널 (output channel) = {out}")
    if max(spans) == 0:
        print("⚠️ L·R 모두 변화 없음 — 마이크 전원(P0.01)·클럭 확인 필요 (no signal on either edge)")
    elif spans[1 - last.out_ch] > 2 * spans[last.out_ch]:
        # 2배 넘게 차이 날 때만 경고 — 마이크가 반대 edge에서 선을 놓으면 값이 유지돼 L≈R로 보임
        # Warn only on a >2x difference; a held DIN line makes L ≈ R
        print(f"⚠️ 다른 채널({'LR'[1 - last.out_ch]}) 변화가 더 큼 → -DPDM_OUT_CH={1 - last.out_ch} 검토")
    return fs


def main() -> None:
    args = parse_args()
    src = SOURCES[args.source]
    col, diags, raw, parser = receive(args.port, args.seconds, src)

    if not diags:
        sys.exit(f"진단 프레임 0개 — acq_fw 플래시, LED 0.5초 깜빡임, 포트, "
                 f"--source({src.name}) 확인 (no diagnostic frames)")

    fs = report_i2s(diags) if src.name == "i2s" else report_pdm(diags)
    last = diags[-1]
    x = col.samples()

    print("\n=== 요약 (Summary) ===")
    print(f"샘플 {len(x)}개 = {len(x) / fs:.2f} s, 프레임 {col.frames}, 손실 샘플 {col.missing}, "
          f"CRC 오류 {parser.crc_errors}, 수신 오류 {last.errors}, 장치 송신 버림 {last.tx_drop}")
    if len(x):
        y = x.astype(float)
        rms = np.std(y)
        print(f"int16 값: 최소 {x.min()}, 최대 {x.max()}, 평균(DC) {y.mean():.1f}, "
              f"DC 제거 RMS {rms:.1f} ({20 * np.log10(max(rms, 1e-9) / 32768):.1f} dBFS)")

    if args.no_save or not len(x):
        return
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"mic_{src.name}_{stamp}" + (f"_{args.note}" if args.note else "")
    out = ROOT / "data" / "raw" / tag
    out.mkdir(parents=True, exist_ok=True)
    (out / "session.bin").write_bytes(bytes(raw))
    save_wav(out / "mic.wav", x, fs)
    with (out / "mic.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sample_idx", "value"])
        base = col.first_idx0 or 0
        w.writerows((base + i, int(v)) for i, v in enumerate(x))
    fig = ROOT / "figures" / f"{tag}.png"
    plot(x, fs, fig, tag)
    print(f"\n저장 (saved): {out}/ (session.bin, mic.wav, mic.csv)\n그림 (figure): {fig}")


def save_wav(path: Path, x: np.ndarray, fs: float) -> None:
    """DC 제거 후 저장 (WAV with DC removed, int16)."""
    y = x.astype(np.float64) - x.mean()
    y = np.clip(np.round(y), -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(int(round(fs)))
        wf.writeframes(y.tobytes())


def plot(x: np.ndarray, fs: float, path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy import signal

    t = np.arange(len(x)) / fs
    f, pxx = signal.welch(x.astype(float) - x.mean(), fs=fs, nperseg=min(2048, len(x)))
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6))
    a1.plot(t, x, lw=0.5)
    a1.set(xlabel="Time [s]", ylabel="int16", title=title)
    a1.grid(alpha=0.3)
    a2.semilogy(f, pxx + 1e-12, lw=1)
    a2.set(xlabel="Frequency [Hz]", ylabel="PSD", xlim=(0, fs / 2))
    a2.grid(alpha=0.3)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    main()

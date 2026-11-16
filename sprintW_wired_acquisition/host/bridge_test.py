"""Gate 0 — XIAO USB 브리지 처리량 측정 (USB-bridge throughput measurement).

apps/bridge_test 펌웨어가 보내는 TEST 프레임(type 0x7F)을 받아
목표 속도 단계별로 송신·수신 속도, 프레임 손실, CRC 오류를 표로 낸다.
Receives TEST frames from apps/bridge_test and reports, per target-rate step,
the device send rate, received rate, lost frames and CRC errors.

예 (Example)::

    .venv/bin/python host/bridge_test.py --port /dev/cu.usbmodem03E4434C3 --baud 460800
    .venv/bin/python host/bridge_test.py --port ... --baud 460800 --append notes/gate0.md --save

펌웨어 단계: 10, 20, 30, 40, 60, 80, 100 kB/s, 최대(0) — 각 10초, 반복
Firmware steps: 10..100 kB/s and max (0), 10 s each, repeating (80 s cycle).
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import serial  # pyserial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from frame import TYPE_TEST, Frame, FrameParser  # noqa: E402

STEP_S = 10.0           # 펌웨어 STEP_MS와 같게 (match firmware STEP_MS)
FULL_STEP_FRACTION = 0.8  # 이보다 짧은 단계는 '부분'으로 표시 (shorter = partial)
TEST_INFO = struct.Struct("<BHII")  # step_idx, target, baud, tx_err
ROOT = Path(__file__).resolve().parents[1]


@dataclass
class StepStats:
    """한 단계(연속 구간)의 통계 (Statistics for one contiguous step)."""

    step_idx: int
    target: int
    baud: int
    frames: int = 0
    bytes_rx: int = 0
    first_idx0: int = 0
    first_idx_last: int = 0
    t0_us: int = 0
    t_last_us: int = 0
    t_wrap_us: int = 0  # uint32 시계가 돌아간 만큼 보정 (wrap correction)
    crc_err0: int = 0
    crc_err1: int = 0
    tx_err0: int = 0
    tx_err1: int = 0
    pc_t0: float = 0.0
    pc_t1: float = 0.0
    sizes: set[int] = field(default_factory=set)

    @property
    def dev_dt_s(self) -> float:
        return (self.t_last_us + self.t_wrap_us - self.t0_us) / 1e6

    @property
    def sent_frames(self) -> int:
        return self.first_idx_last - self.first_idx0 + 1

    @property
    def lost(self) -> int:
        return self.sent_frames - self.frames

    def rates_kbps(self) -> tuple[float, float]:
        """(장치 송신 kB/s, 수신 kB/s) — 기기 시계 기준 (device clock based)."""
        dt = self.dev_dt_s
        if dt <= 0 or self.frames < 2:
            return 0.0, 0.0
        size = max(self.sizes)
        # 간격 수 = 프레임 수 - 1 (intervals = frames - 1)
        sent = (self.sent_frames - 1) * size / dt / 1000
        recv = (self.bytes_rx - size) / dt / 1000
        return sent, recv

    @property
    def partial(self) -> bool:
        return self.dev_dt_s < STEP_S * FULL_STEP_FRACTION


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="예: /dev/cu.usbmodem03E4434C3")
    ap.add_argument("--baud", type=int, required=True,
                    help="펌웨어 빌드 baud와 같아야 함 (must match firmware build)")
    ap.add_argument("--duration", type=float, default=95.0,
                    help="수신 시간 [s], 기본 95 = 한 주기(80 s) + 여유")
    ap.add_argument("--append", type=Path, default=None,
                    help="결과 표를 이 마크다운 파일 끝에 추가 (e.g. notes/gate0.md)")
    ap.add_argument("--save", action="store_true",
                    help="받은 바이트를 data/raw/gate0_<baud>_<시각>.bin 으로 저장")
    return ap.parse_args()


def run(args: argparse.Namespace) -> list[StepStats]:
    parser = FrameParser()
    steps: list[StepStats] = []
    cur: StepStats | None = None
    raw_chunks: list[bytes] = []
    total_rx = 0
    baud_warned = False

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit(f"포트를 열 수 없음 (cannot open port): {exc}")

    with ser:
        ser.reset_input_buffer()
        t_start = time.monotonic()
        t_print = t_start
        while (now := time.monotonic()) - t_start < args.duration:
            try:
                chunk = ser.read(max(1, ser.in_waiting))
            except serial.SerialException as exc:
                print(f"\n수신 오류 (read error): {exc}")
                break
            if not chunk:
                continue
            total_rx += len(chunk)
            if args.save:
                raw_chunks.append(chunk)

            for fr in parser.feed(chunk):
                if fr.type != TYPE_TEST or len(fr.payload) < TEST_INFO.size:
                    continue
                cur = _account(fr, cur, steps, now)
                if cur.baud != args.baud and not baud_warned:
                    print(f"\n⚠️  펌웨어 baud {cur.baud} ≠ --baud {args.baud}")
                    baud_warned = True

            if now - t_print >= 1.0:
                el = now - t_start
                print(f"\r{el:5.1f}s  rx {total_rx / el / 1000:6.1f} kB/s  "
                      f"frames {parser.frames_ok:6d}  crc_err {parser.crc_errors:3d}  "
                      f"step {cur.step_idx if cur else '-'} "
                      f"(target {cur.target if cur else '-'})   ", end="", flush=True)
                t_print = now
    print()

    if args.save and raw_chunks:
        out = ROOT / "data" / "raw" / f"gate0_{args.baud}_{datetime.now():%Y%m%d_%H%M%S}.bin"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(b"".join(raw_chunks))
        print(f"저장 (saved): {out}")

    if parser.frames_ok == 0:
        print("프레임 0개 — baud 일치, 펌웨어 플래시 여부, 포트 확인 (no frames received)")
    print(f"파서 (parser): ok={parser.frames_ok} crc_err={parser.crc_errors} "
          f"skipped_bytes={parser.skipped_bytes}")
    return steps


def _account(fr: Frame, cur: StepStats | None, steps: list[StepStats],
             pc_now: float) -> StepStats:
    """프레임 하나를 단계 통계에 반영 (Add one frame to the step statistics)."""
    step_idx, target, baud, tx_err = TEST_INFO.unpack_from(fr.payload, 0)
    if cur is None or step_idx != cur.step_idx:
        cur = StepStats(step_idx, target, baud,
                        first_idx0=fr.first_idx, t0_us=fr.t_dev_us,
                        crc_err0=fr.crc_errors_before, tx_err0=tx_err, pc_t0=pc_now)
        cur.t_last_us = fr.t_dev_us
        steps.append(cur)
    if fr.t_dev_us < cur.t_last_us:  # uint32 µs 시계 랩 (clock wrap)
        cur.t_wrap_us += 1 << 32
    cur.frames += 1
    cur.bytes_rx += fr.size
    cur.sizes.add(fr.size)
    cur.first_idx_last = fr.first_idx
    cur.t_last_us = fr.t_dev_us
    cur.crc_err1 = fr.crc_errors_before
    cur.tx_err1 = tx_err
    cur.pc_t1 = pc_now
    return cur


def table(steps: list[StepStats], baud: int) -> str:
    lines = [
        f"### baud {baud} — {datetime.now():%Y-%m-%d %H:%M}",
        "",
        "| baud | 목표 kB/s | 송신 kB/s | 실수신 kB/s | 손실 프레임 | CRC 오류 | tx 오류 | 판정 |",
        "|------|-----------|-----------|-------------|-------------|----------|---------|------|",
    ]
    for s in steps:
        if s.partial:
            continue
        sent, recv = s.rates_kbps()
        crc = s.crc_err1 - s.crc_err0
        txe = s.tx_err1 - s.tx_err0
        ok = s.lost == 0 and crc == 0 and txe == 0
        tgt = "최대 (max)" if s.target == 0 else str(s.target)
        lines.append(f"| {baud} | {tgt} | {sent:.1f} | {recv:.1f} | {s.lost} | {crc} | "
                     f"{txe} | {'✅' if ok else '❌'} |")
    lines.append("")
    lines.append("> 송신 = 장치가 실제로 보낸 속도 (baud 한계에 걸리면 목표보다 낮음), "
                 "실수신 = PC가 CRC 통과로 받은 속도. 둘 다 기기 시계 기준.")
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    steps = run(args)
    md = table(steps, args.baud)
    print()
    print(md)
    if args.append:
        path = args.append if args.append.is_absolute() else ROOT / args.append
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as f:
            f.write("\n" + md + "\n")
        print(f"\n추가함 (appended): {path}")


if __name__ == "__main__":
    main()

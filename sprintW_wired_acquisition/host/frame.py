"""장치 → PC 바이너리 프레임 파서 (Device → PC binary frame parser).

계획서 1.3절 형식 (Format from plan section 1.3), little-endian::

    sync(2)=A5 5A | type(1) | len(2) | seq(2) | t_dev_us(4) | first_idx(4)
    | payload(len) | crc(2)

CRC는 CRC-16/XMODEM (poly 0x1021, init 0) — ``sync``부터 ``payload`` 끝까지.
CRC is CRC-16/XMODEM over sync..payload; device uses ``crc16_itu_t(0, ...)``.

Gate 0(Day 1)와 Day 4 로거가 같이 쓴다 / Shared by Gate 0 and the Day 4 logger.
"""

from __future__ import annotations

import binascii
import struct
from dataclasses import dataclass, field

SYNC = b"\xa5\x5a"
HEADER = struct.Struct("<2sBHHII")  # sync, type, len, seq, t_dev_us, first_idx
HEADER_LEN = HEADER.size  # 15
CRC_LEN = 2
MAX_PAYLOAD = 4096  # 이보다 크면 가짜 sync로 보고 버림 (larger → treat as false sync)

TYPE_PPG = 1
TYPE_MIC = 2
TYPE_EVENT = 3
TYPE_STATUS = 4
TYPE_MIC_PDM = 5  # 내장 PDM 마이크 (on-board PDM mic)
TYPE_IMU = 6  # 내장 IMU 6축 (on-board IMU, 6-axis)
TYPE_PPG_DIAG = 0x7B  # MAX30102 진단 (PPG diagnostics)
TYPE_IMU_DIAG = 0x7C  # 내장 IMU 진단 (on-board IMU diagnostics)
TYPE_PDM_DIAG = 0x7D  # 내장 마이크 진단 (on-board mic diagnostics)
TYPE_MIC_DIAG = 0x7E  # Day 3 마이크 진단 (mic diagnostics)
TYPE_TEST = 0x7F
KNOWN_TYPES = {TYPE_PPG, TYPE_MIC, TYPE_EVENT, TYPE_STATUS, TYPE_MIC_PDM, TYPE_IMU,
               TYPE_PPG_DIAG, TYPE_IMU_DIAG, TYPE_PDM_DIAG, TYPE_MIC_DIAG, TYPE_TEST}


def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM. ``crc16_xmodem(b"123456789") == 0x31C3``."""
    return binascii.crc_hqx(data, 0)


@dataclass(frozen=True)
class Frame:
    """파싱된 프레임 하나 (One parsed frame)."""

    type: int
    seq: int
    t_dev_us: int
    first_idx: int
    payload: bytes
    # 이 프레임이 나올 때까지의 파서 CRC 오류 누적 수 (parser CRC-error count so far)
    crc_errors_before: int = field(default=0, compare=False)

    @property
    def size(self) -> int:
        """sync~crc 전체 바이트 수 (Total bytes on the wire)."""
        return HEADER_LEN + len(self.payload) + CRC_LEN


def build_frame(ftype: int, seq: int, t_dev_us: int, first_idx: int, payload: bytes) -> bytes:
    """프레임 바이트 만들기 — 테스트·시뮬레이션용 (Build frame bytes for tests)."""
    head = HEADER.pack(SYNC, ftype, len(payload), seq & 0xFFFF,
                       t_dev_us & 0xFFFFFFFF, first_idx & 0xFFFFFFFF)
    body = head + payload
    return body + struct.pack("<H", crc16_xmodem(body))


class FrameParser:
    """바이트 스트림 → 프레임 (Byte stream → frames), 재동기화 포함 (with resync).

    사용법 (Usage)::

        parser = FrameParser()
        for frame in parser.feed(chunk):
            ...

    통계 (Counters):
        crc_errors: CRC가 틀린 프레임 수 (frames rejected by CRC)
        skipped_bytes: sync를 찾느라 버린 바이트 수 (bytes discarded while resyncing)
        frames_ok: 정상 프레임 수 (good frames)
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.skipped_bytes = 0
        self.frames_ok = 0

    def feed(self, data: bytes) -> list[Frame]:
        """새 바이트를 넣고 완성된 프레임 목록을 돌려준다 (Feed bytes, get frames)."""
        self._buf += data
        out: list[Frame] = []
        buf = self._buf

        while True:
            idx = buf.find(SYNC)
            if idx < 0:
                # 마지막 바이트가 sync 첫 바이트일 수 있으니 남겨 둠
                # Keep a trailing 0xA5 that may start the next sync
                keep = 1 if buf[-1:] == SYNC[:1] else 0
                drop = len(buf) - keep
                self.skipped_bytes += drop
                del buf[:drop]
                break
            if idx > 0:
                self.skipped_bytes += idx
                del buf[:idx]
            if len(buf) < HEADER_LEN:
                break

            _, ftype, plen, seq, t_us, first_idx = HEADER.unpack_from(buf, 0)
            if plen > MAX_PAYLOAD or ftype not in KNOWN_TYPES:
                # 가짜 sync (payload 안의 A5 5A 등) → 1바이트 건너뛰고 다시 찾기
                # False sync → skip one byte and search again
                self.skipped_bytes += 1
                del buf[:1]
                continue

            total = HEADER_LEN + plen + CRC_LEN
            if len(buf) < total:
                break

            body = bytes(buf[:HEADER_LEN + plen])
            (crc_rx,) = struct.unpack_from("<H", buf, HEADER_LEN + plen)
            if crc16_xmodem(body) != crc_rx:
                self.crc_errors += 1
                self.skipped_bytes += 1
                del buf[:1]  # 이 sync는 버리고 다음 sync부터 (resync after this sync)
                continue

            out.append(Frame(ftype, seq, t_us, first_idx, body[HEADER_LEN:],
                             self.crc_errors))
            self.frames_ok += 1
            del buf[:total]

        return out

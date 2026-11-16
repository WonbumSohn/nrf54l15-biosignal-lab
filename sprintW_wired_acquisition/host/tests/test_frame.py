"""host/frame.py 테스트 (Tests for the frame parser).

실행 (Run): ``cd sprintW_wired_acquisition && .venv/bin/pytest host -q``
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from frame import (  # noqa: E402
    TYPE_PPG,
    TYPE_TEST,
    FrameParser,
    build_frame,
    crc16_xmodem,
)


def _test_frames(n: int, plen: int = 240) -> list[bytes]:
    return [
        build_frame(TYPE_TEST, seq=i, t_dev_us=1000 * i, first_idx=i,
                    payload=bytes((i + k) & 0xFF for k in range(plen)))
        for i in range(n)
    ]


def test_crc_check_vector() -> None:
    """CRC-16/XMODEM 표준 검증 벡터 (standard check value)."""
    assert crc16_xmodem(b"123456789") == 0x31C3


def test_parse_clean_stream() -> None:
    p = FrameParser()
    frames = p.feed(b"".join(_test_frames(5)))
    assert [f.seq for f in frames] == [0, 1, 2, 3, 4]
    assert frames[2].t_dev_us == 2000
    assert frames[2].payload[0] == 2
    assert p.crc_errors == 0 and p.skipped_bytes == 0


def test_byte_by_byte_feed() -> None:
    """한 바이트씩 들어와도 같은 결과 (same result when fed 1 byte at a time)."""
    p = FrameParser()
    out = []
    for b in b"".join(_test_frames(3)):
        out += p.feed(bytes([b]))
    assert [f.seq for f in out] == [0, 1, 2]


def test_corrupted_frame_rejected() -> None:
    """일부러 깨뜨린 프레임은 거부, 나머지는 살림 (corrupted frame rejected)."""
    raw = _test_frames(3)
    bad = bytearray(raw[1])
    bad[50] ^= 0xFF
    p = FrameParser()
    frames = p.feed(raw[0] + bytes(bad) + raw[2])
    assert [f.seq for f in frames] == [0, 2]
    assert p.crc_errors == 1


def test_resync_from_middle() -> None:
    """프레임 중간부터 읽기 시작해도 다음 프레임에서 재동기화 (resync mid-stream)."""
    stream = b"".join(_test_frames(4))
    p = FrameParser()
    frames = p.feed(stream[100:])  # 첫 프레임 중간부터 (start inside frame 0)
    assert [f.seq for f in frames] == [1, 2, 3]
    assert p.skipped_bytes > 0


def test_false_sync_in_payload() -> None:
    """payload 안의 A5 5A가 프레임 시작으로 오인되지 않음 (false sync in payload)."""
    payload = b"\xa5\x5a" + bytes(range(30))
    raw = build_frame(TYPE_PPG, 7, 123, 456, payload)
    p = FrameParser()
    frames = p.feed(b"\x00\xa5" + raw + raw)
    assert len(frames) == 2
    assert frames[0].seq == 7 and frames[0].payload == payload


def test_partial_then_rest() -> None:
    raw = _test_frames(1)[0]
    p = FrameParser()
    assert p.feed(raw[:10]) == []
    assert len(p.feed(raw[10:])) == 1

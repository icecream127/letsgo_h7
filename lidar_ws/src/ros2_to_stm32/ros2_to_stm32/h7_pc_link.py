"""USART3 PC Link V1 for the H743 firmware."""
import binascii
import secrets
import time
from typing import NamedTuple

import serial

MAX_WIRE = 32
REPLY_TIMEOUT = 0.15


def encode_frame(body: bytes) -> bytes:
    crc = binascii.crc_hqx(body, 0xFFFF)
    return body + f'*{crc:04X}\n'.encode('ascii')


def decode_frame(wire: bytes) -> bytes | None:
    if not (7 <= len(wire) <= MAX_WIRE and wire.endswith(b'\n')):
        return None
    body = wire[:-6]
    return body if encode_frame(body) == wire else None


class PcStatus(NamedTuple):
    state: int
    gait: int
    phase: int
    fault: int
    online_mask: int
    remote_locked: int


class PcLink:
    def __init__(self, device):
        self.port = serial.Serial(device, baudrate=115200, timeout=0.02,
                                  write_timeout=0.05, exclusive=True)
        self.buffer = bytearray()
        self.session = None
        self.sequence = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        self.port.close()

    def _request(self, body, reply_prefix, reply_length):
        wire = encode_frame(body)
        if self.port.write(wire) != len(wire):
            raise OSError('USART3 发送不完整')
        deadline = time.monotonic() + REPLY_TIMEOUT
        while time.monotonic() < deadline:
            self.buffer.extend(self.port.read(min(128, self.port.in_waiting or 1)))
            while (end := self.buffer.find(b'\n')) >= 0:
                frame = bytes(self.buffer[:end + 1])
                del self.buffer[:end + 1]
                reply = decode_frame(frame)
                if reply and len(reply) == reply_length and reply.startswith(reply_prefix):
                    return reply
            if len(self.buffer) > MAX_WIRE:
                self.buffer.clear()
        raise TimeoutError(f'USART3 未收到 {body[:2].decode()} 对应的回包')

    def connect(self):
        nonce = f'{secrets.randbelow(0xFFFFFFFF) + 1:08X}'.encode('ascii')
        reply = self._request(b'H1' + nonce, b'Y1' + nonce, 18)
        session = int(reply[10:18], 16)
        if not session or reply[10:18] != f'{session:08X}'.encode('ascii'):
            raise ValueError('USART3 返回了无效的会话')
        self.session = session
        self.sequence = 0

    def _operation(self, operation, reply_type, reply_length):
        self.sequence = (self.sequence + 1) & 0xFFFF
        identity = f'1{self.session:08X}{self.sequence:04X}'.encode('ascii')
        reply = self._request(operation + identity, reply_type + identity, reply_length)
        fields = reply[14:]
        if any(byte not in b'0123456789ABCDEF' for byte in fields):
            raise ValueError('USART3 回包含无效字段')
        return tuple(int(fields[i:i + 2], 16) for i in range(0, len(fields), 2))

    def status(self):
        return PcStatus(*self._operation(b'K', b'U', 26))

    def start_trot(self):
        result, = self._operation(b'T', b'A', 16)
        if result != 1:  # PENDING: firmware owns the request and is starting.
            raise OSError(f'H743 拒绝 TROT，回执 {result}')

    def stand(self):
        result, = self._operation(b'Z', b'A', 16)
        if result != 0:
            raise OSError(f'H743 未确认本会话的收步请求，回执 {result}')

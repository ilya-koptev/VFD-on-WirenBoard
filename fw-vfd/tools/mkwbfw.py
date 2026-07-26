"""Собрать .wbfw из голого bin приложения.

Формат: инфоблок 32 байта + тело, добитое 0xFF до кратности 136. Флешер ВБ
читает файл 16-битными словами, меняет в них байты местами, а Modbus передаёт
регистры старшим байтом вперёд — на линии байты идут в том же порядке, что и в
файле, поэтому тело кладём как есть.

    python mkwbfw.py app.bin app.wbfw
"""
import binascii
import struct
import sys

SIGNATURE = b"vfd128x32"
SIG_LEN = 12
INFO_SIZE = 32
CHUNK = 136
FORMAT_VERSION = 1

if len(sys.argv) != 3:
    raise SystemExit(__doc__)

body = open(sys.argv[1], "rb").read()
if not body:
    raise SystemExit("пустой bin")

pad = (-len(body)) % CHUNK
body_padded = body + b"\xFF" * pad

info = SIGNATURE.ljust(SIG_LEN, b"\0")
info += struct.pack(">I", len(body_padded))          # размер того, что реально уйдёт
info += struct.pack(">I", binascii.crc32(body_padded) & 0xFFFFFFFF)
info += bytes([FORMAT_VERSION])
info += b"\0" * (INFO_SIZE - len(info))
assert len(info) == INFO_SIZE

open(sys.argv[2], "wb").write(info + body_padded)
print("%s: тело %d байт + %d добивки = %d, кусков по %d: %d"
      % (sys.argv[2], len(body), pad, len(body_padded), CHUNK, len(body_padded) // CHUNK))
print("подпись %s, CRC32 %08X" % (SIGNATURE.decode(), binascii.crc32(body_padded) & 0xFFFFFFFF))

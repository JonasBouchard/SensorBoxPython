#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from smbus2 import SMBus, i2c_msg

I2C_BUS = 0           # <-- Bus 0 (GPIO0/1)
SGP30_ADDR = 0x58

def _crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

def _tx_cmd(bus, cmd: int):
    # envoie exactement 2 octets (MSB, LSB)
    msg = i2c_msg.write(SGP30_ADDR, bytes([(cmd >> 8) & 0xFF, cmd & 0xFF]))
    bus.i2c_rdwr(msg)

def _rx_words(bus, n_words: int) -> list[int]:
    # lit n_words * (2 bytes + 1 CRC)
    rd = i2c_msg.read(SGP30_ADDR, n_words * 3)
    bus.i2c_rdwr(rd)
    raw = bytes(rd)
    out = []
    for i in range(0, len(raw), 3):
        hi, lo, crc = raw[i], raw[i+1], raw[i+2]
        if _crc8(bytes([hi, lo])) != crc:
            raise IOError("CRC invalide (SGP30)")
        out.append((hi << 8) | lo)
    return out

def _read_words(bus, cmd: int, n_words: int, delay_s: float = 0.02) -> list[int]:
    _tx_cmd(bus, cmd)
    time.sleep(delay_s)
    return _rx_words(bus, n_words)

def main():
    with SMBus(I2C_BUS) as bus:
        time.sleep(0.1)  # petite marge après l'alim

        # 1) Feature set (sanity check)
        feat = _read_words(bus, 0x202F, 1)[0]
        print(f"Feature set : 0x{feat:04X}")

        # 2) Numéro de série (3 words = 48 bits)
        serial = _read_words(bus, 0x3682, 3)
        print(f"Serial      : {serial[0]:04X}-{serial[1]:04X}-{serial[2]:04X}")

        # 3) Init IAQ
        _tx_cmd(bus, 0x2003)
        time.sleep(0.05)
        print("IAQ init OK. Les ~15 premières mesures servent de rodage…")

        # 4) Boucle de mesures IAQ
        t0 = time.time()
        while True:
            eco2, tvoc = _read_words(bus, 0x2008, 2, delay_s=0.05)  # eCO2 ppm, TVOC ppb
            uptime = time.time() - t0
            print(f"[{uptime:6.1f}s] eCO2={eco2:4d} ppm | TVOC={tvoc:4d} ppb")
            time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nArrêt.")

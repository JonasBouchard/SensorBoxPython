#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from smbus2 import SMBus, i2c_msg

# =======================
#   Paramètres I²C
# =======================
I2C_BUS   = 0
SGP30_ADDR = 0x58
AHT20_ADDR = 0x38
BMP280_ADDR = 0x77

# =======================
#   Utilitaires SGP30
# =======================
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

# =======================
#   AHT20 (T & RH)
# =======================
def aht20_init(bus):
    # Soft reset
    try:
        bus.write_byte(AHT20_ADDR, 0xBA)
        time.sleep(0.02)
    except Exception:
        pass
    # Init (normalement pas strictement nécessaire, mais sûr)
    msg = i2c_msg.write(AHT20_ADDR, [0xBE, 0x08, 0x00])
    bus.i2c_rdwr(msg)
    time.sleep(0.01)

def aht20_read(bus):
    # Trigger measurement
    msg = i2c_msg.write(AHT20_ADDR, [0xAC, 0x33, 0x00])
    bus.i2c_rdwr(msg)
    time.sleep(0.08)  # 80 ms

    # Read 6 bytes
    rd = i2c_msg.read(AHT20_ADDR, 6)
    bus.i2c_rdwr(rd)
    d = bytes(rd)
    # d[0] status, puis 20 bits RH, puis 20 bits Temp
    raw_rh = ((d[1] << 12) | (d[2] << 4) | (d[3] >> 4)) & 0xFFFFF
    raw_t  = (((d[3] & 0x0F) << 16) | (d[4] << 8) | d[5]) & 0xFFFFF

    rh = (raw_rh / (1<<20)) * 100.0
    t  = (raw_t  / (1<<20)) * 200.0 - 50.0
    return t, rh

# =======================
#   BMP280 (P & T)
#   Implémentation minimale (calibration + compensation)
# =======================
class BMP280:
    def __init__(self, bus, addr=BMP280_ADDR):
        self.bus = bus
        self.addr = addr
        # Vérifier ID (0x58 pour BMP280, 0x60 pour BME280)
        chip_id = self._read_u8(0xD0)
        if chip_id not in (0x58, 0x60):
            raise IOError(f"BMP/BME280 non détecté (ID=0x{chip_id:02X}) à 0x{addr:02X}")

        # Reset
        self._write_u8(0xE0, 0xB6)
        time.sleep(0.005)

        # Lire coefficients de calibration
        self.dig_T1 = self._read_u16le(0x88)
        self.dig_T2 = self._read_s16le(0x8A)
        self.dig_T3 = self._read_s16le(0x8C)
        self.dig_P1 = self._read_u16le(0x8E)
        self.dig_P2 = self._read_s16le(0x90)
        self.dig_P3 = self._read_s16le(0x92)
        self.dig_P4 = self._read_s16le(0x94)
        self.dig_P5 = self._read_s16le(0x96)
        self.dig_P6 = self._read_s16le(0x98)
        self.dig_P7 = self._read_s16le(0x9A)
        self.dig_P8 = self._read_s16le(0x9C)
        self.dig_P9 = self._read_s16le(0x9E)

        # ctrl_meas: oversampling x1 pour T et P, mode normal
        self._write_u8(0xF4, (1 << 5) | (1 << 2) | 3)  # osrs_t=1, osrs_p=1, mode=3
        # config: standby 250 ms, IIR filter off (peut ajuster)
        self._write_u8(0xF5, (5 << 5) | (0 << 2) | 0)

        self.t_fine = 0

    # --- I/O helpers ---
    def _read_u8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def _write_u8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read_u16le(self, reg):
        lo = self._read_u8(reg)
        hi = self._read_u8(reg+1)
        return (hi << 8) | lo

    def _read_s16le(self, reg):
        val = self._read_u16le(reg)
        return val - 65536 if val & 0x8000 else val

    def _read_raw(self):
        # press_msb..xlsb (0xF7..0xF9), temp_msb..xlsb (0xFA..0xFC)
        data = self.bus.read_i2c_block_data(self.addr, 0xF7, 6)
        adc_p = ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4)) & 0xFFFFF
        adc_t = ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4)) & 0xFFFFF
        return adc_t, adc_p

    def read(self):
        adc_t, adc_p = self._read_raw()

        # ----- Temp compensation -----
        var1 = (((adc_t >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
        var2 = (((((adc_t >> 4) - self.dig_T1) * ((adc_t >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8
        temp_c = temp / 100.0

        # ----- Pressure compensation -----
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            return temp_c, None  # évite division par 0
        p = (1048576 - adc_p)
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (self.dig_P8 * p) >> 19
        p = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)
        press_pa = p / 256.0  # en Pascal
        return temp_c, press_pa

# =======================
#   Programme principal
# =======================
def main():
    with SMBus(I2C_BUS) as bus:
        time.sleep(0.1)  # petite marge après l'alim

        # ----- SGP30 : checks + init -----
        feat = _read_words(bus, 0x202F, 1)[0]
        print(f"SGP30 Feature set : 0x{feat:04X}")

        serial = _read_words(bus, 0x3682, 3)
        print(f"SGP30 Serial      : {serial[0]:04X}-{serial[1]:04X}-{serial[2]:04X}")

        _tx_cmd(bus, 0x2003)  # init IAQ
        time.sleep(0.05)
        print("SGP30 IAQ init OK. Les ~15 premières mesures servent de rodage…")

        # ----- AHT20 : init -----
        try:
            aht20_init(bus)
            # Test lecture pour valider rapidement
            t_aht, rh = aht20_read(bus)
            print(f"AHT20 prêt. T={t_aht:.2f}°C  RH={rh:.1f}%")
        except Exception as e:
            print(f"AHT20 init/lecture : {e}")
            t_aht, rh = None, None

        # ----- BMP280 : init -----
        bmp = None
        try:
            bmp = BMP280(bus, BMP280_ADDR)
            t_bmp, p_pa = bmp.read()
            p_hpa = p_pa / 100.0 if p_pa is not None else None
            print(f"BMP280 prêt.  T={t_bmp:.2f}°C  P={p_hpa:.2f} hPa" if p_hpa is not None else
                  f"BMP280 prêt.  T={t_bmp:.2f}°C  P=—")
        except Exception as e:
            print(f"BMP280 init/lecture : {e}")

        # ----- Boucle mesures -----
        t0 = time.time()
        while True:
            # SGP30
            eco2, tvoc = _read_words(bus, 0x2008, 2, delay_s=0.05)  # eCO2 ppm, TVOC ppb

            # AHT20
            t_aht, rh = (None, None)
            try:
                t_aht, rh = aht20_read(bus)
            except Exception:
                pass

            # BMP280
            t_bmp, p_pa = (None, None)
            if bmp:
                try:
                    t_bmp, p_pa = bmp.read()
                except Exception:
                    pass

            # Agrégation simple pour T (si dispo) : priorité BMP puis AHT
            temp_c = None
            if t_bmp is not None:
                temp_c = t_bmp
            elif t_aht is not None:
                temp_c = t_aht

            # Affichage
            uptime = time.time() - t0
            line = [f"[{uptime:6.1f}s] eCO2={eco2:4d} ppm | TVOC={tvoc:4d} ppb"]
            if temp_c is not None:
                line.append(f"T={temp_c:.2f}°C")
            if rh is not None:
                line.append(f"RH={rh:.1f}%")
            if p_pa is not None:
                line.append(f"P={p_pa/100.0:.2f} hPa")
            print(" | ".join(line))

            time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nArrêt.")

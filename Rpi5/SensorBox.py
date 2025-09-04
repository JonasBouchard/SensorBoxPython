#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from smbus2 import SMBus, i2c_msg

# =======================
#   I²C Parameters
# =======================
I2C_BUS    = 0
SGP30_ADDR = 0x58
AHT20_ADDR = 0x38
BMP280_ADDR = 0x77

# =======================
#   SGP30 Utilities
# =======================
def _crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

def _tx_cmd(bus, cmd: int):
    msg = i2c_msg.write(SGP30_ADDR, bytes([(cmd >> 8) & 0xFF, cmd & 0xFF]))
    bus.i2c_rdwr(msg)

def _rx_words(bus, n_words: int) -> list[int]:
    rd = i2c_msg.read(SGP30_ADDR, n_words * 3)
    bus.i2c_rdwr(rd)
    raw = bytes(rd)
    out = []
    for i in range(0, len(raw), 3):
        hi, lo, crc = raw[i], raw[i+1], raw[i+2]
        if _crc8(bytes([hi, lo])) != crc:
            raise IOError("Invalid CRC (SGP30)")
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
    try:
        bus.write_byte(AHT20_ADDR, 0xBA)  # soft reset
        time.sleep(0.02)
    except Exception:
        pass
    msg = i2c_msg.write(AHT20_ADDR, [0xBE, 0x08, 0x00])
    bus.i2c_rdwr(msg)
    time.sleep(0.01)

def aht20_read(bus):
    msg = i2c_msg.write(AHT20_ADDR, [0xAC, 0x33, 0x00])
    bus.i2c_rdwr(msg)
    time.sleep(0.08)
    rd = i2c_msg.read(AHT20_ADDR, 6)
    bus.i2c_rdwr(rd)
    d = bytes(rd)
    raw_rh = ((d[1] << 12) | (d[2] << 4) | (d[3] >> 4)) & 0xFFFFF
    raw_t  = (((d[3] & 0x0F) << 16) | (d[4] << 8) | d[5]) & 0xFFFFF
    rh = (raw_rh / (1<<20)) * 100.0
    t  = (raw_t  / (1<<20)) * 200.0 - 50.0
    return t, rh

# =======================
#   BMP280 (P & T)
# =======================
class BMP280:
    def __init__(self, bus, addr=BMP280_ADDR):
        self.bus = bus
        self.addr = addr
        chip_id = self._read_u8(0xD0)
        if chip_id not in (0x58, 0x60):
            raise IOError(f"BMP/BME280 not detected (ID=0x{chip_id:02X}) at 0x{addr:02X}")
        self._write_u8(0xE0, 0xB6)
        time.sleep(0.005)
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
        self._write_u8(0xF4, (1 << 5) | (1 << 2) | 3)  # osrs_t=1, osrs_p=1, mode normal
        self._write_u8(0xF5, (5 << 5) | (0 << 2) | 0)  # t_standby=250ms
        self.t_fine = 0

    def _read_u8(self, reg):  return self.bus.read_byte_data(self.addr, reg)
    def _write_u8(self, reg, val): self.bus.write_byte_data(self.addr, reg, val)
    def _read_u16le(self, reg):
        lo = self._read_u8(reg); hi = self._read_u8(reg+1); return (hi << 8) | lo
    def _read_s16le(self, reg):
        val = self._read_u16le(reg); return val - 65536 if val & 0x8000 else val

    def _read_raw(self):
        data = self.bus.read_i2c_block_data(self.addr, 0xF7, 6)
        adc_p = ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4)) & 0xFFFFF
        adc_t = ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4)) & 0xFFFFF
        return adc_t, adc_p

    def read(self):
        adc_t, adc_p = self._read_raw()
        var1 = (((adc_t >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
        var2 = (((((adc_t >> 4) - self.dig_T1) * ((adc_t >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2
        temp_c = ((self.t_fine * 5 + 128) >> 8) / 100.0
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            return temp_c, None
        p = (1048576 - adc_p)
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (self.dig_P8 * p) >> 19
        p = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)
        press_pa = p / 256.0
        return temp_c, press_pa

# =======================
#   PMS5003 (PM)
# =======================
# Required pins (BCM): RESET=2, SET=3, TXD=14, RXD=15
# Requires: pyserial + gpiozero (often preinstalled on Raspberry Pi OS)
import serial
try:
    from gpiozero import DigitalOutputDevice
except Exception:
    DigitalOutputDevice = None  # handle without GPIO control

class PMS5003:
    def __init__(self, port="/dev/serial0", baud=9600, pin_set=3, pin_reset=2):
        self.port = port
        self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)
        self.enable = None
        self.reset = None
        if DigitalOutputDevice is not None:
            self.enable = DigitalOutputDevice(pin_set, active_high=True, initial_value=True)
            self.reset  = DigitalOutputDevice(pin_reset, active_high=True, initial_value=True)
            self.wake()
            self.reset_pulse()
    # clear the buffer
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def wake(self):
        if self.enable: self.enable.on()

    def sleep(self):
        if self.enable: self.enable.off()

    def reset_pulse(self):
        if self.reset:
            self.reset.off()
            time.sleep(0.05)
            self.reset.on()
            time.sleep(0.1)

    def _read_frame(self, timeout_s=1.2):
        start = time.time()
        s = self.ser
        while time.time() - start < timeout_s:
            b = s.read(1)
            if len(b) == 0:
                continue
            if b[0] != 0x42:
                continue
            b2 = s.read(1)
            if len(b2) == 0:
                continue
            if b2[0] != 0x4D:
                continue
            rest = s.read(30)  # total 32 bytes, already read 2
            if len(rest) != 30:
                continue
            frame = bytes([0x42, 0x4D]) + rest
            # checksum
            chk = (frame[-2] << 8) | frame[-1]
            calc = sum(frame[:-2]) & 0xFFFF
            if chk != calc:
                continue
            return frame
        raise TimeoutError("PMS5003: frame read timeout")

    def read(self):
        f = self._read_frame()
        length = (f[2] << 8) | f[3]    # expected 28
        # Relevant fields (µg/m3)
        pm1_cf1   = (f[4] << 8) | f[5]
        pm25_cf1  = (f[6] << 8) | f[7]
        pm10_cf1  = (f[8] << 8) | f[9]
        pm1_atm   = (f[10] << 8) | f[11]
        pm25_atm  = (f[12] << 8) | f[13]
        pm10_atm  = (f[14] << 8) | f[15]
        # counts (number of particles / 0.1L of air)
        n03 = (f[16] << 8) | f[17]
        n05 = (f[18] << 8) | f[19]
        n10 = (f[20] << 8) | f[21]
        n25 = (f[22] << 8) | f[23]
        n50 = (f[24] << 8) | f[25]
        n100= (f[26] << 8) | f[27]
        return {
            "pm1": pm1_atm, "pm25": pm25_atm, "pm10": pm10_atm,
            "pm1_cf1": pm1_cf1, "pm25_cf1": pm25_cf1, "pm10_cf1": pm10_cf1,
            "n0_3": n03, "n0_5": n05, "n1_0": n10, "n2_5": n25, "n5_0": n50, "n10_0": n100,
            "length": length,
        }

# =======================
#   Main program
# =======================
def main():
    with SMBus(I2C_BUS) as bus:
        time.sleep(0.1)

        # ----- SGP30: checks + init -----
        feat = _read_words(bus, 0x202F, 1)[0]
        print(f"SGP30 Feature set : 0x{feat:04X}")
        serial = _read_words(bus, 0x3682, 3)
        print(f"SGP30 Serial      : {serial[0]:04X}-{serial[1]:04X}-{serial[2]:04X}")
        _tx_cmd(bus, 0x2003)  # init IAQ
        time.sleep(0.05)
        print("SGP30 IAQ init OK. The first ~15 measurements serve as burn-in…")

        # ----- AHT20 -----
        try:
            aht20_init(bus)
            t_aht, rh = aht20_read(bus)
            print(f"AHT20 ready. T={t_aht:.2f}°C  RH={rh:.1f}%")
        except Exception as e:
            print(f"AHT20 init/read: {e}")
            t_aht, rh = None, None

        # ----- BMP280 -----
        bmp = None
        try:
            bmp = BMP280(bus, BMP280_ADDR)
            t_bmp, p_pa = bmp.read()
            p_hpa = p_pa / 100.0 if p_pa is not None else None
            print(f"BMP280 ready.  T={t_bmp:.2f}°C  P={p_hpa:.2f} hPa" if p_hpa is not None else
                  f"BMP280 ready.  T={t_bmp:.2f}°C  P=—")
        except Exception as e:
            print(f"BMP280 init/read: {e}")

        # ----- PMS5003 -----
        pms = None
        try:
            pms = PMS5003(port="/dev/ttyAMA0", baud=9600, pin_set=3, pin_reset=2)
            sample = pms.read()
            print(f"PMS5003 ready. PM1={sample['pm1']} PM2.5={sample['pm25']} PM10={sample['pm10']} µg/m³")
        except Exception as e:
            print(f"PMS5003 init/read: {e}")

        # ----- Measurement loop -----
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

            # PMS5003
            pm1 = pm25 = pm10 = None
            if pms:
                try:
                    s = pms.read()
                    pm1, pm25, pm10 = s["pm1"], s["pm25"], s["pm10"]
                except Exception:
                    pass

            # Aggregated temperature
            temp_c = t_bmp if t_bmp is not None else t_aht

            uptime = time.time() - t0
            line = [f"[{uptime:6.1f}s] eCO2={eco2:4d} ppm | TVOC={tvoc:4d} ppb"]
            if temp_c is not None: line.append(f"T={temp_c:.2f}°C")
            if rh is not None:     line.append(f"RH={rh:.1f}%")
            if p_pa is not None:   line.append(f"P={p_pa/100.0:.2f} hPa")
            if pm25 is not None:   line.append(f"PM1={pm1} PM2.5={pm25} PM10={pm10} µg/m³")
            print(" | ".join(line))

            time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple Flask web UI for displaying SensorBox readings.

This interface exposes sensor data via a small web server. The root page
shows the most recent readings and refreshes automatically every few seconds.
A JSON representation is also available at ``/data``.
"""

from __future__ import annotations

import time
import threading
import json
import re
from pathlib import Path
from flask import Flask, render_template, jsonify
from smbus2 import SMBus

from SensorBox import (
    _read_words,
    _tx_cmd,
    aht20_init,
    aht20_read,
    BMP280,
    PMS5003,
    I2C_BUS,
)

app = Flask(__name__)
DATA_FILE = Path(__file__).resolve().parent / "latest.json"

# ------------------------ Sensor initialisation -------------------------
bus = SMBus(I2C_BUS)
_tx_cmd(bus, 0x2003)  # initialise SGP30 IAQ measurement

try:
    aht20_init(bus)
except Exception:
    pass  # sensor may be missing

time.sleep(0.05)

try:
    bmp = BMP280(bus)
except Exception:  # sensor not present
    bmp = None

try:
    pms = PMS5003(port="/dev/ttyAMA0", baud=9600, pin_set=3, pin_reset=2)
except Exception:  # sensor not present
    pms = None


latest: dict[str, str] = {
    "eCO2": "—",
    "TVOC": "—",
    "Temperature": "—",
    "Relative Humidity": "—",
    "Pressure": "—",
    "PM1": "—",
    "PM2.5": "—",
    "PM10": "—",
}
_lock = threading.Lock()

THRESHOLDS = {
    "eCO2": 1000,  # ppm
    "TVOC": 500,  # ppb
    "Temperature": 30,  # °C
    "Relative Humidity": 60,  # %
    "PM1": 35,  # µg/m³
    "PM2.5": 25,  # µg/m³
    "PM10": 50,  # µg/m³
}


def _apply_thresholds(data: dict[str, str]) -> None:
    for key, limit in THRESHOLDS.items():
        if key in data:
            match = re.search(r"[-+]?\d*\.?\d+", data[key])
            if match and float(match.group()) > limit:
                data[key] += " DANGER"


def _poll_sensors() -> None:
    """Background thread periodically polling sensors."""
    while True:
        data: dict[str, str] = {}

        try:
            eco2, tvoc = _read_words(bus, 0x2008, 2, delay_s=0.05)
            data["eCO2"] = f"{eco2} ppm"
            data["TVOC"] = f"{tvoc} ppb"
        except Exception:
            pass

        try:
            t_aht, rh = aht20_read(bus)
        except Exception:
            t_aht, rh = None, None

        t_bmp = p_pa = None
        if bmp:
            try:
                t_bmp, p_pa = bmp.read()
            except Exception:
                pass

        temp_c = t_bmp if t_bmp is not None else t_aht
        if temp_c is not None:
            data["Temperature"] = f"{temp_c:.2f} °C"
        if rh is not None:
            data["Relative Humidity"] = f"{rh:.1f}%"
        if p_pa is not None:
            data["Pressure"] = f"{p_pa/1000:.2f} kPa"

        if pms:
            try:
                s = pms.read()
                data["PM1"] = f"{s['pm1']} µg/m³"
                data["PM2.5"] = f"{s['pm25']} µg/m³"
                data["PM10"] = f"{s['pm10']} µg/m³"
            except Exception:
                pass
        else:
            # Fallback: read particulate data written by SensorBox.py
            try:
                with DATA_FILE.open(encoding="utf-8") as fh:
                    d = json.load(fh)
                if "PM1" in d:
                    data["PM1"] = f"{d['PM1']} µg/m³"
                if "PM2.5" in d:
                    data["PM2.5"] = f"{d['PM2.5']} µg/m³"
                if "PM10" in d:
                    data["PM10"] = f"{d['PM10']} µg/m³"
            except Exception:
                pass

        _apply_thresholds(data)

        with _lock:
            latest.update(data)

        time.sleep(1.0)


threading.Thread(target=_poll_sensors, daemon=True).start()


def get_latest() -> dict[str, str]:
    """Return a snapshot of the most recent sensor data."""
    with _lock:
        return dict(latest)




@app.route("/")
def index():
    return render_template("index.html", data=get_latest())


@app.route("/data")
def data():
    return jsonify(get_latest())


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=8888)
    finally:
        try:
            bus.close()
        except Exception:
            pass

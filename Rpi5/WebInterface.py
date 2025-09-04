#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple Flask web UI for displaying SensorBox readings.

This interface exposes sensor data via a small web server. The root page
shows the most recent readings and refreshes automatically every few seconds.
A JSON representation is also available at ``/data``.
"""

from __future__ import annotations

import time
from flask import Flask, render_template_string, jsonify
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


def read_sensors() -> dict[str, str]:
    """Poll available sensors and return a mapping of readings."""
    data: dict[str, str] = {}

    # Gas sensor
    try:
        eco2, tvoc = _read_words(bus, 0x2008, 2, delay_s=0.05)
        data["eCO2"] = f"{eco2} ppm"
        data["TVOC"] = f"{tvoc} ppb"
    except Exception:
        pass

    # Temperature & humidity from AHT20
    try:
        t_aht, rh = aht20_read(bus)
    except Exception:
        t_aht, rh = None, None

    # Temperature & pressure from BMP280
    t_bmp = p_pa = None
    if bmp:
        try:
            t_bmp, p_pa = bmp.read()
        except Exception:
            pass

    temp_c = t_bmp if t_bmp is not None else t_aht
    if temp_c is not None:
        data["Temp"] = f"{temp_c:.1f} °C"
    if rh is not None:
        data["RH"] = f"{rh:.1f}%"
    if p_pa is not None:
        data["Pressure"] = f"{p_pa/1000:.1f} kPa"

    # Particulate matter from PMS5003
    if pms:
        try:
            s = pms.read()
            data["PM1"] = f"{s['pm1']} µg/m³"
            data["PM2.5"] = f"{s['pm25']} µg/m³"
            data["PM10"] = f"{s['pm10']} µg/m³"
        except Exception:
            pass

    return data


# Simple template. The meta refresh tag reloads the page every 5 seconds.
TEMPLATE = """
<!doctype html>
<title>SensorBox</title>
<meta http-equiv="refresh" content="5">
<h1>SensorBox readings</h1>
<ul>
{% for key, val in data.items() %}
  <li><strong>{{ key }}:</strong> {{ val }}</li>
{% endfor %}
</ul>
"""


@app.route("/")
def index():
    data = read_sensors()
    return render_template_string(TEMPLATE, data=data)


@app.route("/data")
def data():
    return jsonify(read_sensors())


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=8000)
    finally:
        try:
            bus.close()
        except Exception:
            pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple Tkinter GUI for displaying SensorBox readings on an HDMI screen.

This interface is designed for use on the Raspberry Pi with an attached HDMI
monitor.  It periodically queries the sensors defined in ``SensorBox.py`` and
shows their readings in a window.  The goal is to provide a quick visual
overview of the environment without relying on the command line.
"""

import time
import tkinter as tk
import sys
from smbus2 import SMBus

# Reuse sensor utilities and classes from the existing SensorBox module
from SensorBox import (
    _read_words,
    _tx_cmd,
    aht20_init,
    aht20_read,
    BMP280,
    PMS5003,
    I2C_BUS,
)


class Interface(tk.Tk):
    """Tkinter window displaying live sensor readings."""

    def __init__(self) -> None:
        super().__init__()
        self.title("SensorBox")
        self.geometry("320x240")  # fits small HDMI screens nicely

        # Create labels for each metric
        self.labels: dict[str, tk.Label] = {}
        metrics = [
            "eCO2",
            "TVOC",
            "Temp",
            "RH",
            "Pressure",
            "PM1",
            "PM2.5",
            "PM10",
        ]
        for name in metrics:
            lbl = tk.Label(self, text=f"{name}: --", font=("Arial", 14))
            lbl.pack(anchor="w")
            self.labels[name] = lbl

        # ----- Sensor initialisation -----
        self.bus = SMBus(I2C_BUS)
        _tx_cmd(self.bus, 0x2003)  # initialise SGP30 IAQ measurement
        time.sleep(0.05)

        try:
            aht20_init(self.bus)
        except Exception:
            pass  # sensor may be missing

        try:
            self.bmp = BMP280(self.bus)
        except Exception:
            self.bmp = None

        try:
            self.pms = PMS5003(port="/dev/ttyAMA0", baud=9600, pin_set=3, pin_reset=2)
        except Exception:
            self.pms = None

        self.after(1000, self.update_readings)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------
    def update_readings(self) -> None:
        """Poll sensors and refresh GUI labels."""
        try:
            eco2, tvoc = _read_words(self.bus, 0x2008, 2, delay_s=0.05)
            self.labels["eCO2"].config(text=f"eCO2: {eco2} ppm")
            self.labels["TVOC"].config(text=f"TVOC: {tvoc} ppb")
        except Exception:
            pass

        # Temperature & humidity from AHT20
        try:
            t_aht, rh = aht20_read(self.bus)
        except Exception:
            t_aht, rh = None, None

        # Temperature & pressure from BMP280
        t_bmp = p_pa = None
        if self.bmp:
            try:
                t_bmp, p_pa = self.bmp.read()
            except Exception:
                pass

        temp_c = t_bmp if t_bmp is not None else t_aht
        if temp_c is not None:
            self.labels["Temp"].config(text=f"Temp: {temp_c:.1f} °C")
        if rh is not None:
            self.labels["RH"].config(text=f"RH: {rh:.1f}%")
        if p_pa is not None:
            self.labels["Pressure"].config(text=f"Pressure: {p_pa/1000:.1f} kPa")

        # Particulate matter from PMS5003
        pm1 = pm25 = pm10 = None
        if self.pms:
            try:
                s = self.pms.read()
                pm1, pm25, pm10 = s["pm1"], s["pm25"], s["pm10"]
            except Exception:
                pass
        if pm1 is not None:
            self.labels["PM1"].config(text=f"PM1: {pm1} µg/m³")
        if pm25 is not None:
            self.labels["PM2.5"].config(text=f"PM2.5: {pm25} µg/m³")
        if pm10 is not None:
            self.labels["PM10"].config(text=f"PM10: {pm10} µg/m³")

        # Schedule next update
        self.after(1000, self.update_readings)

    # ------------------------------------------------------------------
    def on_close(self) -> None:
        """Close sensors and exit."""
        try:
            self.bus.close()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    try:
        app = Interface()
        app.mainloop()
    except tk.TclError as err:
        print(
            "Error: Unable to start Tkinter. Ensure a graphical display is attached "
            "or X11 forwarding is enabled."
        )
        print(f"TclError: {err}")
        sys.exit(1)
# SensorBox-Python

## Raspberry Pi 5 pre-configuration

Follow these steps to prepare a Raspberry Pi 5 before running the SensorBox scripts and connecting the sensors.

1. **Update the system**

   ```bash
   sudo apt update
   sudo apt full-upgrade -y
   ```

2. **Install required packages**

   ```bash
   sudo apt install -y python3-pip python3-smbus i2c-tools git
   pip3 install smbus2 gpiozero pyserial flask
   ```

3. **Enable interfaces**

   Launch `raspi-config` and activate the hardware interfaces used by the sensors:

   ```bash
   sudo raspi-config
   ```

   - *Interface Options → I2C* → **Enable**
   - *Interface Options → Serial Port* → **Disable** the login shell and **Enable** the serial hardware

4. **Edit the boot configuration**

   Add the overlays required for I²C bus 0 and for the UART port:

   ```bash
   sudo nano /boot/firmware/config.txt
   ```

   Append the following lines at the end of the file then save:

   ```
   dtoverlay=i2c0
   enable_uart=1
   ```

   Reboot to apply the changes:

   ```bash
   sudo reboot
   ```

5. **Verify I²C bus 0**

   After reboot, ensure that the connected sensors appear on bus 0:

   ```bash
   i2cdetect -y -r 0
   ```

   You should see addresses listed for any devices connected via SDA/SCL.


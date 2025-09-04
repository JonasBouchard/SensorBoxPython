# SensorBox-Python

## Requirements :
to make communication between Raspberry Pi 5 and some sensors (sgp30, aht20, bmp280), you first need to slightly edit your ```/boot/firmware/config.txt``` file by adding ```dtoverlay=i2c0``` at the end and then reboot.

```
sudo nano /boot/firmware/config.txt
sudo reboot
```

To see if it worked, you will have to enter the following command and check if your Raspberry Pi recognizes any device connected through SDA and SCL :

```
i2cdetect -y -r 0
```
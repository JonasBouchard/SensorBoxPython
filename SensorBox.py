from machine import I2C, Pin
from Modules import ahtx0 , bmp280 , sgp30
import time

# Initialize I2C communication
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)

# Configure AHT20 + BMP280 sensor
bmp = bmp280.BMP280(i2c)
aht = ahtx0.AHT10(i2c)
sgp = sgp30.SGP30(i2c)

while True:
    temperature = round((bmp.temperature + aht.temperature) / 2 , 2)
    humidity = round(aht.relative_humidity , 2)
    pressure = round(bmp.pressure/1000 , 2)
    co2eq = round(sgp.iaq_measure()[0] , 2)
    tvoc = round(sgp.iaq_measure()[1] , 2)

    # Print temperature and pressure data
    print("Temperature : " + str(temperature) + "°C")
    print("Humidity :    " + str(humidity) + "%")
    print("Pressure :    " + str(pressure) + "kPa")
    print("CO2eq :       " + str(co2eq) + "ppm")
    print("total VOCs :  " + str(tvoc) + "ppb\n\n\n")

    # Read data every second
    time.sleep_ms(100)
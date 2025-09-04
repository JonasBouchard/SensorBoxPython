from machine import I2C, Pin, UART
import ahtx0, bmp280 , sgp30, pms5003
import time

# Initialize I2C communication
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)

# Configure AHT20 + BMP280 sensor
bmp = bmp280.BMP280(i2c)
aht = ahtx0.AHT10(i2c)
sgp = sgp30.SGP30(i2c)
pms = pms5003.PMS5003(uart=UART(0, tx=Pin(16), rx=Pin(17), baudrate=9600),pin_enable=Pin(3),pin_reset=Pin(2),mode="active",)

while True:
    temperature = round((bmp.temperature + aht.temperature) / 2 , 2)
    humidity = round(aht.relative_humidity , 2)
    pressure = round(bmp.pressure/1000 , 2)
    
    co2eq = round(sgp.iaq_measure()[0] , 2)
    tvoc = round(sgp.iaq_measure()[1] , 2)
    
    pms_data = pms.read()
    

    # Print temperature and pressure data
    print("\nTemperature : " + str(temperature) + "°C")
    print("Humidity :    " + str(humidity) + "%")
    print("Pressure :    " + str(pressure) + "kPa")
    print("CO2eq :       " + str(co2eq) + "ppm")
    print("total VOCs :  " + str(tvoc) + "ppb")
    print("PM1.0 :       " + str(pms_data.pm_ug_per_m3(1.0, atmospheric_environment=True)) + "ug/m3")
    print("PM2.5 :       " + str(pms_data.pm_ug_per_m3(2.5, atmospheric_environment=True)) + "ug/m3")
    print("PM10 :        " + str(pms_data.pm_ug_per_m3(10, atmospheric_environment=True)) + "ug/m3")

    # Read data every 5 seconds
    time.sleep_ms(5000)

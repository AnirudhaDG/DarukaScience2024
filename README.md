# Science Sensor Array Firmware

This repo runs all the science sensors used in our sensor array.

## Sensors

### LTR390 Adafruit UV Sensor
* Product page: https://www.adafruit.com/product/4831
* Adafruit tutorial: https://learn.adafruit.com/adafruit-ltr390-uv-sensor
* Datasheet: https://optoelectronics.liteon.com/upload/download/DS86-2015-0004/LTR-390UV_Final_%20DS_V1%201.pdf
* Protocol: I2C
* Voltage: 5V
* Library: https://github.com/levkovigor/LTR390.git

### BME280 Temperature, Humidity, Pressure Sensor
* Product page: https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
* Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
* Protocol: I2C
* Voltage: 5V
* Library: https://github.com/finitespace/BME280.git

### AGS02MA Air Quality Sensor
* Product page: http://www.aosong.com/m/en/products-33.html
* Datasheet: https://cdn-shop.adafruit.com/product-files/5593/datasheet+ags02ma.pdf
* Protocol: I2C
* Voltage: **3.3V**
* Library: https://github.com/RobTillaart/AGS02MA.git

### MQ-9 Gas Sensor (Carbon Monoxide, Methane, LPG)
* Product page: https://robu.in/product/mq-9-carbon-monoxide-methane-lpg-gas-sensor-module/
* Datasheet: https://robu.in/wp-content/uploads/2017/09/MQ-9-Datasheet-ROBU.IN_.pdf
* Protocol: Analog
* Voltage: 5V

### MQ-136 Gas Sensor (Hydrogen Sulfide)
* Product page: https://robu.in/product/mq-136-hydrogen-sulfide-gas-sensor-module/
* Datasheet: http://www.sensorica.ru/pdf/MQ-136.pdf
* Protocol: Analog
* Voltage: 5V

### NPK Soil Sensor
* Library: https://github.com/4-20ma/ModbusMaster.git
* Protocol: RS485
* Voltage: 12V
* Datasheet: [attached](./npk-datasheet.pdf). This is not exactly the same sensor, but it's the best lead we have.

### MAX485 RS485 Module (for NPK Soil Sensor)
* IC product page: https://www.analog.com/en/products/max485.html
* Module product page: https://robu.in/product/max485-ttl-rs485/
* Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX1487-MAX491.pdf
* Protocol: RS485/TTL
* Voltage: 5V

## Data format
The data is returned on a Serial port every 20 seconds. It is formatted as a comma-separated string.

Sequence of data:

| Index | Sensor | Data Type | Units |
| --- | --- | --- | --- |
| 1 | BME280 | Temperature | °C |
| 2 | BME280 | Humidity | % |
| 3 | BME280 | Pressure | Pa |
| 4 | BME280 | Altitude | m |
| 5 | AGS02MA | TVOC | ppb |
| 6 | LTR390 | Value type | 0 = Ambient Light, 1 = UV Index |
| 7 | LTR390 | Value | lux or UV index |
| 8 | MQ-9 | Value | (analogue; 0-1023) |
| 9 | MQ-136 | Value | (analogue; 0-1023) |
| 10 | NPK | Temperature | °C |
| 11 | NPK | Moisture | % |
| 12 | NPK | Electrical Conductivity | µS/cm |
| 13 | NPK | pH | pH |
| 14 | NPK | Nitrogen | mg/kg |
| 15 | NPK | Phosphorus | mg/kg |
| 16 | NPK | Potassium | mg/kg |
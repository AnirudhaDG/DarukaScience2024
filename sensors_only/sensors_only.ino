#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#define FAILING_DELAY 1e3
#define SERIAL_DELAY 1e2
#define LOOP_DELAY 5e3
#define SERIAL_BAUD_RATE 115200
#define MQ136_PIN A0
#define MQ9_PIN A1
#define SERVO_PIN 3

long int itertaion = 0;

// LTR390 Adafruit UV Sensor
#include <LTR390.h>
#define UV_I2C_ADDRESS 0x53
LTR390 UVSensor = LTR390(UV_I2C_ADDRESS);

// AGS02MA Adafruit Gas Sensor
#include <AGS02MA.h>
#define GAS_I2C_ADDRESS 26
AGS02MA GasSensor = AGS02MA(GAS_I2C_ADDRESS);

// BME280 Bosch Environmental Sensor
#include <BME280I2C.h>
BME280I2C barometer;



// Servo
Servo servo;

// BME280 Bosch Environmental Sensor
void configureBME280(BME280I2C* barometer) {
    Serial.println("Initializing BME280");

    delay(SERIAL_DELAY);

    while (!barometer->begin()) {
        Serial.println("BME280 initialization unsuccessful");
        delay(FAILING_DELAY);
    }

    Serial.println("BME280 initialization successful");
}

void readBME280Data(Stream* client, BME280I2C* barometer) {
    float temp(NAN), hum(NAN), pres(NAN);

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);

    barometer->read(pres, temp, hum, tempUnit, presUnit);

    Serial.println("*********************************************************");
    Serial.println("BME280 Sensor (pressure, temperature, humidity, altitude)");
    Serial.println("*********************************************************");
    client->print("Temp: ");
    client->print(temp);
    client->print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
    client->print("\t\tHumidity: ");
    client->print(hum);
    client->print("% RH");
    client->print("\t\tPressure: ");
    client->print(pres);
    client->print("Pa");

    float altitude = 44330 * (1.0 - pow((pres / 100) / 1013, 0.1903));
    client->print("\t\t% Alt: ");
    client->println(altitude);
    Serial.println();
}

// void readBME280Data(BME280I2C barometer) {
//     float pressure, temperature, humidity;

//     BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
//     BME280::PresUnit presUnit(BME280::PresUnit_Pa);

//     barometer.read(pressure, temperature, humidity, tempUnit, presUnit);

//     Serial.println("***********************************************");
//     Serial.println("BME280 Sensor (pressure, temperature, humidity)");
//     Serial.println("***********************************************");
//     Serial.print("Pressure: ");
//     Serial.print(pressure);
//     Serial.print(" Pa, ");
//     Serial.print("Temperature: ");
//     Serial.print(temperature);
//     Serial.print(" °C, ");
//     Serial.print("Humidity: ");
//     Serial.print(humidity);
//     Serial.println(" %");
//     Serial.println();
// }

// AGS02MA Adafruit Gas Sensor
void configureAGS02MAGasSensor(AGS02MA gasObject) {
    Serial.println("Initializing Gas Sensor");

    delay(SERIAL_DELAY);

    while (!gasObject.begin()) {
        Serial.println("Gas Sensor initialization unsuccessful");
        delay(FAILING_DELAY);
    }

    gasObject.setPPBMode();

    Serial.println("Gas Sensor initialization successful");
}

void readAGS02MAGasSensor(AGS02MA gasObject) {
    Serial.println("*********************");
    Serial.println("AGS02MA Sensor (TVOC)");
    Serial.println("*********************");
    Serial.print("Gas Sensor Value: ");
    Serial.println(gasObject.readPPB());
    Serial.println();
}

// Analog Gas Sensor
void readGasAnalogSensor(uint8_t pin) {
    long value = analogRead(pin);

    Serial.print("Gas Sensor Value: ");
    Serial.println(value);
    Serial.println();
}

// LTR390 Adafruit UV Sensor
void configureUV(LTR390 uvObject) {
    Serial.println("Initializing UV Sensor");

    delay(SERIAL_DELAY);

    while (!uvObject.init()) {
        Serial.println("UV Sensor initialization unsuccessful");
        delay(FAILING_DELAY);
    }

    Serial.println("UV Sensor initialization successful");

    uvObject.setMode(LTR390_MODE_ALS);
    uvObject.setGain(LTR390_GAIN_3);
    uvObject.setResolution(LTR390_RESOLUTION_18BIT);
}

void readUVData(LTR390 uvObject) {
    Serial.println("*******************************************");
    Serial.println("LTR390 Sensor (UV Index, Ambient Light Lux)");
    Serial.println("*******************************************");

    if (uvObject.newDataAvailable()) {
        if (uvObject.getMode() == LTR390_MODE_ALS) {
            Serial.print("Ambient Light Lux: ");
            Serial.println(uvObject.getLux());
            uvObject.setGain(LTR390_GAIN_18);                 // Recommended for UVI - x18
            uvObject.setResolution(LTR390_RESOLUTION_20BIT);  // Recommended for UVI - 20-bit
            uvObject.setMode(LTR390_MODE_UVS);
        } else if (uvObject.getMode() == LTR390_MODE_UVS) {
            Serial.print("UV Index: ");
            Serial.println(uvObject.getUVI());
            uvObject.setGain(LTR390_GAIN_3);                  // Recommended for Lux - x3
            uvObject.setResolution(LTR390_RESOLUTION_18BIT);  // Recommended for Lux - 18-bit
            uvObject.setMode(LTR390_MODE_ALS);
        }
    }
    Serial.println();
}

void setup() {
    Wire.begin();
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
    }  // Wait

    configureBME280(&barometer);
    configureAGS02MAGasSensor(GasSensor);
    configureUV(UVSensor);

    servo.attach(SERVO_PIN);
}

void loop() {
    Serial.println("\n\n");
    Serial.print("Iteration: ");
    Serial.println(itertaion++);

    readBME280Data(&Serial, &barometer);
    readAGS02MAGasSensor(GasSensor);
    readUVData(UVSensor);

    Serial.println("********************************");
    Serial.println("MQ9 Sensor (CO, CH4, H2, C2H5OH)");
    Serial.println("********************************");
    readGasAnalogSensor(MQ9_PIN);

    Serial.println("******************");
    Serial.println("MQ136 Sensor (H2S)");
    Serial.println("******************");
    readGasAnalogSensor(MQ136_PIN);

    for (uint8_t angle = 0; angle < 255; angle++) {
        servo.write(angle);
        delay(15);
    }

    for (uint8_t angle = 255; angle > 0; angle--) {
        servo.write(angle);
        delay(15);
    }
}
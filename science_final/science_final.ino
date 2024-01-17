#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#define FAILING_DELAY 1e3  // how long to wait before retrying to initialize a sensor
#define SENSOR_DELAY 20e3  // how long to wait between sensor readings
#define NPK_BAUD_RATE 9600
#define SERIAL_BAUD_RATE 9600

// LTR390 Adafruit UV Sensor
#include <LTR390.h>
#define UV_I2C_ADDRESS 0x53
LTR390 UVSensor = LTR390(UV_I2C_ADDRESS);

void configureUV();
void readUVData();

// AGS02MA Adafruit Gas Sensor
#include <AGS02MA.h>
#define GAS_I2C_ADDRESS 26
AGS02MA gasSensor = AGS02MA(GAS_I2C_ADDRESS);

void configureAGS02MAGasSensor();
void readAGS02MAGasSensor();

// BME280 Bosch Environmental Sensor
#include <BME280I2C.h>
BME280I2C barometer;

void configureBME280();
void readBME280Data();

// Analog Gas Sensors
uint8_t MQ136_PIN = A0;
uint8_t MQ9_PIN = A1;

// NPK Sensor
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

const uint8_t RO_PIN = 2;
const uint8_t DI_PIN = 3;
const uint8_t RE_PIN = 7;
const uint8_t DE_PIN = 6;

SoftwareSerial swSerial(RO_PIN, DI_PIN);  // Receive (data in) pin, Transmit (data out) pin
ModbusMaster NPKObject;

void preTransmission();
void postTransmission();
void configureNPK();
void readNPK();

void setup() {
    Wire.begin();

    Serial.begin(SERIAL_BAUD_RATE);

    while (!Serial) {
    }  // Wait
    configureBME280();
    configureAGS02MAGasSensor();
    configureUV();
    configureNPK();
}

int counter = 0;
unsigned long sensorTimer = millis();

void loop() {
    if (millis() >= sensorTimer + SENSOR_DELAY) {

        // DO NOT CHANGE THE ORDER OF THESE FUNCTIONS
        readBME280Data();
        readAGS02MAGasSensor();
        readUVData();
        readGasAnalogSensor(MQ9_PIN);
        readGasAnalogSensor(MQ136_PIN);
        readNPK();
        sensorTimer = millis();
    }
}

// Put the MAX485 into transmit mode
void preTransmission() {
    digitalWrite(RE_PIN, 1);
    digitalWrite(DE_PIN, 1);
}

// Put the MAX485 into receive mode
void postTransmission() {
    digitalWrite(RE_PIN, 0);
    digitalWrite(DE_PIN, 0);
}

void configureNPK() {
    // configure the MAX485 RE & DE control signals and enable receive mode
    pinMode(RE_PIN, OUTPUT);
    pinMode(DE_PIN, OUTPUT);
    digitalWrite(DE_PIN, 0);
    digitalWrite(RE_PIN, 0);

    // Modbus communication runs at 9600 baud
    swSerial.begin(NPK_BAUD_RATE);

    // Modbus slave ID of NPK sensor is 1
    NPKObject.begin(1, swSerial);

    // Callbacks to allow us to set the RS485 Tx/Rx direction
    NPKObject.preTransmission(preTransmission);
    NPKObject.postTransmission(postTransmission);
}

void readNPK() {
    uint8_t result;
    Serial.print("NPK");
    Serial.print(",");

    // remove any characters from the receive buffer
    // ask for 7x 16-bit words starting at register address 0x0000
    result = NPKObject.readHoldingRegisters(0x0000, 7);

    if (result == NPKObject.ku8MBSuccess) {
        Serial.print(NPKObject.getResponseBuffer(0) / 10.0);  // temprature in C
        Serial.print(",");

        Serial.print(NPKObject.getResponseBuffer(1) / 10.0);  // moisture %
        Serial.print(",");

        Serial.print(NPKObject.getResponseBuffer(2));  // EC in uS/cm
        Serial.print(",");

        Serial.print(NPKObject.getResponseBuffer(3) / 100.0);  // pH
        Serial.print(",");

        Serial.print(NPKObject.getResponseBuffer(4) * 10);  // N in mg/kg
        Serial.print(",");

        Serial.print(NPKObject.getResponseBuffer(5) * 10);  // P in mg/kg
        Serial.print(",");

        // This sensor has println because it is the last one
        Serial.println(NPKObject.getResponseBuffer(6) * 10);  // K in mg/kg
    } else {
        Serial.println("Error");
    }
}

// BME280 Bosch Environmental Sensor
void configureBME280() {
    while (!barometer.begin()) {
        delay(FAILING_DELAY);
    }
}

void readBME280Data() {
    float temp(NAN), hum(NAN), pres(NAN);

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);

    barometer.read(pres, temp, hum, tempUnit, presUnit);

    Serial.print(temp);
    Serial.print(",");
    Serial.print(hum);
    Serial.print(",");
    Serial.print(pres);
    Serial.print(",");  // in Pa

    float altitude = 44330 * (1.0 - pow((pres / 100) / 1013, 0.1903));
    Serial.print(altitude);
    Serial.print(",");
    ;
}

// AGS02MA Adafruit Gas Sensor
void configureAGS02MAGasSensor() {
    while (!gasSensor.begin()) {
        delay(FAILING_DELAY);
    }

    gasSensor.setPPBMode();
}

void readAGS02MAGasSensor() {
    Serial.print(gasSensor.readPPB());
    Serial.print(",");
}

// Analog Gas Sensor
void readGasAnalogSensor(uint8_t pin) {
    long value = analogRead(pin);
    Serial.print(value);
    Serial.print(",");
}

// LTR390 Adafruit UV Sensor
void configureUV() {
    while (!UVSensor.init()) {
        delay(FAILING_DELAY);
    }
    UVSensor.setMode(LTR390_MODE_ALS);
    UVSensor.setGain(LTR390_GAIN_3);
    UVSensor.setResolution(LTR390_RESOLUTION_18BIT);
}

void readUVData() {
    if (UVSensor.newDataAvailable()) {
        if (UVSensor.getMode() == LTR390_MODE_ALS) {
            Serial.print("0.0");
            Serial.print(",");
            Serial.println(UVSensor.getLux());
            UVSensor.setGain(LTR390_GAIN_18);                 // Recommended for UVI - x18
            UVSensor.setResolution(LTR390_RESOLUTION_20BIT);  // Recommended for UVI - 20-bit
            UVSensor.setMode(LTR390_MODE_UVS);
        } else if (UVSensor.getMode() == LTR390_MODE_UVS) {
            Serial.print("1.0");
            Serial.print(",");
            Serial.println(UVSensor.getUVI());
            UVSensor.setGain(LTR390_GAIN_3);                  // Recommended for Lux - x3
            UVSensor.setResolution(LTR390_RESOLUTION_18BIT);  // Recommended for Lux - 18-bit
            UVSensor.setMode(LTR390_MODE_ALS);
        }
    }
}
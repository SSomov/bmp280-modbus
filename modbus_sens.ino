//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <math.h>

Adafruit_BME280 bme; // I2C

long temp = 0;
long humi = 0;
long pres = 0;
long tr = 0;
void setup() {
    Serial.begin(57600);
    //Serial.println(F("BME280 test"));
    ModbusRTUServer.begin(4, 57600);
    ModbusRTUServer.configureInputRegisters(0x00, 10);
    if (! bme.begin()) {
        // Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
}


void loop() {    
    ModbusRTUServer.poll();
    temp = bme.readTemperature()*10;
    pres = (long)round(bme.readPressure()*7.50062/100.0);
    humi = bme.readHumidity()*10;
    ModbusRTUServer.inputRegisterWrite(0  , 1);
    ModbusRTUServer.inputRegisterWrite(1  , temp);
    ModbusRTUServer.inputRegisterWrite(2  , 1);
    ModbusRTUServer.inputRegisterWrite(3  , humi);
    //tr = long(dewPoint(temp/ 10.0, humi/ 10.0)*10);
    ModbusRTUServer.inputRegisterWrite(4  , tr);
    ModbusRTUServer.inputRegisterWrite(5  , 1);
    ModbusRTUServer.inputRegisterWrite(6  , pres);
    
}

double dewPoint(float celsius, float humidity)
{
  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);
  // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;
  // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

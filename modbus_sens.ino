/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10


Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

#include <Modbusino.h>

/* Initialize the slave with the ID 1 */
ModbusinoSlave modbusino_slave(1);
/* Allocate a mapping of 10 values */
uint16_t tab_reg[10];

    union converter{
float f_val;
uint16_t u_val;
};

union converter a;

void setup() {
    /*Serial.begin(9600);
    Serial.println(F("BME280 test"));*/

    modbusino_slave.setup(115200);
    if (! bme.begin()) {
        // Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

  /*  Serial.println("-- Default Test --");
    Serial.println("normal mode, 16x oversampling for all, filter off,");
    Serial.println("0.5ms standby period");*/
    delayTime = 5000;
    /*
        // инициализация Timer1
    cli(); // отключить глобальные прерывания
    TCCR1A = 0; // установить TCCR1A регистр в 0
    TCCR1B = 0;

    // включить прерывание Timer1 overflow:
    TIMSK1 = (1 << TOIE1);
    // Установить CS10 бит так, чтобы таймер работал при тактовой частоте:
    TCCR1B |= (1 << CS10);
    TCCR1B |= (1 << CS12);

    sei();  // включить глобальные прерывания
    */
    
    // For more details on the following scenarious, see chapter
    // 3.5 "Recommended modes of operation" in the datasheet
    
/*
    // weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
                      
    // suggested rate is 1/60Hz (1m)
    delayTime = 60000; // in milliseconds
*/

/*    
    // humidity sensing
    Serial.println("-- Humidity Sensing Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 0x pressure oversampling");
    Serial.println("= pressure off, filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_NONE, // pressure
                    Adafruit_BME280::SAMPLING_X1,   // humidity
                    Adafruit_BME280::FILTER_OFF );
                      
    // suggested rate is 1Hz (1s)
    delayTime = 1000;  // in milliseconds
*/

/*    
    // indoor navigation
    Serial.println("-- Indoor Navigation Scenario --");
    Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
    Serial.println("0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
    
    // suggested rate is 25Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
    // T_ovs = 2
    // P_ovs = 16
    // H_ovs = 1
    // = 40ms (25Hz)
    // with standby time that should really be 24.16913... Hz
    delayTime = 41;
    */
    
    /*
    // gaming
    Serial.println("-- Gaming Scenario --");
    Serial.println("normal mode, 4x pressure / 1x temperature / 0x humidity oversampling,");
    Serial.println("= humidity off, 0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_X4,   // pressure
                    Adafruit_BME280::SAMPLING_NONE, // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
                      
    // Suggested rate is 83Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5)
    // T_ovs = 1
    // P_ovs = 4
    // = 11.5ms + 0.5ms standby
    delayTime = 12;
*/

    //Serial.println();
}


void loop() {
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme.takeForcedMeasurement(); // has no effect in normal mode
    
    printValues();
    //delay(delayTime);
}


void printValues() {
  /*  Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    Serial.println(int(bme.readTemperature()+0.51));
    Serial.println(int(bme.readTemperature()+0.51), HEX);
    Serial.print(String(int(bme.readTemperature()+0.51), HEX));
    
    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();*/

    a.f_val = bme.readTemperature();

        /* Initialize the first register to have a value to read */
    tab_reg[0] = a.u_val;
    /* Launch Modbus slave loop with:
       - pointer to the mapping
       - max values of mapping */
    modbusino_slave.loop(tab_reg, 10);
}


ISR(TIMER1_OVF_vect)
{
    a.f_val = bme.readTemperature();

        /* Initialize the first register to have a value to read */
    tab_reg[0] = a.u_val;
}

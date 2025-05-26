/***************************************************************************************************/
/*
   This is an Arduino library for 14-bit MAX31855 K-Thermocouple to Digital Converter
   with 12-bit Cold Junction Compensation conneted to hardware 5Mhz SPI with maximum sampling
   rate ~9..10Hz.

   - MAX31855 maximum power supply voltage is 3.6v
   - Maximum SPI bus speed 5Mhz
   - K-type thermocouples have an absolute accuracy of around ±2°C..±6°C.
   - Measurement tempereture range -200°C..+700°C ±2°C or -270°C..+1372°C ±6°C
     with 0.25°C resolution/increment.
   - Cold junction compensation range -40°C..+125° ±3°C with 0.062°C resolution/increment.
     Optimal performance of cold junction compensation happends when the thermocouple cold junction
     & the MAX31855 are at the same temperature. Avoid placing heat-generating devices or components
     near the converter because this may produce an errors.
   - It is strongly recommended to add a 10nF/0.01mF ceramic surface-mount capacitor, placed across
     the T+ and T- pins, to filter noise on the thermocouple lines.
     
   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/MAX31855

   This sensor uses SPI bus to communicate, specials pins are required to interface
   Board:                                    MOSI        MISO        SCLK         SS, don't use for CS   Level
   Uno, Mini, Pro, ATmega168, ATmega328..... 11          12          13           10                     5v
   Mega, Mega2560, ATmega1280, ATmega2560... 51          50          52           53                     5v
   Due, SAM3X8E............................. ICSP4       ICSP1       ICSP3        x                      3.3v
   Leonardo, ProMicro, ATmega32U4........... 16          14          15           x                      5v
   Blue Pill, STM32F103xxxx boards.......... PA17        PA6         PA5          PA4                    3v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO13/D7   GPIO12/D6   GPIO14/D5    GPIO15/D8*             3v/5v
   ESP32.................................... GPIO23/D23  GPIO19/D19  GPIO18/D18   x                      3v

                                             *most boards has 10-12kOhm pullup-up resistor on GPIO2/D4 & GPIO0/D3
                                              for flash & boot

   Frameworks & Libraries:
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/stm32duino/Arduino_Core_STM32
                         - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include <time.h>
#include <cstring>
#include <algorithm>
#include "MAX31855.h"

//#include <iostream>

/**************************************************************************/
/*
    MAX31855()

    Constructor for hardware read only SPI

    NOTE:
    - cs is chip select, set cs low to enable serial interface
*/
/**************************************************************************/
//MAX31855::MAX31855(uint8_t cs)
//{
//  _cs = cs; //cs chip select
//}

namespace max31855 {

    MAX31855::MAX31855(std::shared_ptr<mcp32017::MCP23017> mcp) : mcp23017(mcp) {
    }

    MAX31855::~MAX31855() {
        spi_free(&_spi);
    }
/**************************************************************************/
/*
    begin()

    Initializes & configures hardware SPI
*/
/**************************************************************************/
    void MAX31855::begin(void) {

        int retv = spi_init(&_spi,
                            SPI_DEVICE, // filename like "/dev/spidev0.0"
                            0,          // SPI_* (look "linux/spi/spidev.h")
                            0,          // bits per word (usually 8)
                            5000000);   // max speed [Hz]

        //SPI.begin();                              //setting hardware SCK, MOSI, SS to output, pull SCK, MOSI low & SS high

        delay(MAX31855_CONVERSION_POWER_UP_TIME);

    }

/**************************************************************************/
/*
    detectThermocouple()

    Checks if thermocouple is open, shorted to GND, shorted to VCC

    Return:
    - 0 OK
    - 1 short to VCC
    - 2 short to GND
    - 3 not connected

    NOTE:
    - bit D16 is normally low & goes high if thermocouple is open, shorted to GND or VCC
    - bit D2  is normally low & goes high to indicate a hermocouple short to VCC
    - bit D1  is normally low & goes high to indicate a thermocouple short to GND
    - bit D0  is normally low & goes high to indicate a thermocouple open circuit
*/
/**************************************************************************/
    uint8_t MAX31855::detectThermocouple(int32_t rawValue) {
        if (rawValue == MAX31855_FORCE_READ_DATA) rawValue = readRawData();

        if (rawValue == 0) return MAX31855_THERMOCOUPLE_READ_FAIL;

        if (mcp23017->bitRead(rawValue, 16) == 1) {
            if (mcp23017->bitRead(rawValue, 2) == 1) return MAX31855_THERMOCOUPLE_SHORT_TO_VCC;
            else if (mcp23017->bitRead(rawValue, 1) == 1) return MAX31855_THERMOCOUPLE_SHORT_TO_GND;
            else if (mcp23017->bitRead(rawValue, 0) == 1) return MAX31855_THERMOCOUPLE_NOT_CONNECTED;
            else return MAX31855_THERMOCOUPLE_UNKNOWN;
        }
        return MAX31855_THERMOCOUPLE_OK;
    }

/**************************************************************************/
/*
    getChipID()

    Checks chip ID

    NOTE:
    - bit D17, D3 always return zero & can be used as device ID
*/
/**************************************************************************/
    uint16_t MAX31855::getChipID(int32_t rawValue) {
        if (rawValue == MAX31855_FORCE_READ_DATA) rawValue = readRawData();

        if (rawValue == 0) return MAX31855_THERMOCOUPLE_READ_FAIL;
        if (mcp23017->bitRead(rawValue, 17) == 0 && mcp23017->bitRead(rawValue, 3) == 0) return MAX31855_ID;

        return MAX31855_ERROR;
    }

/**************************************************************************/
/*
    getTemperature()

    Reads Temperature, C

    NOTE:
    - range -200°C..+700°C ±2°C or -270°C..+1372°C ±6°C with 0.25°C
      resolution/increment
    - thermocouple temperature data is 14-bit long
    - bit D31 is the thermocouple temperature sign bit "+" is high & "-" is low,
      if T+ and T- are unconnected it goes low
    - bits D30..D18 contain the converted temperature in the order of MSB to LSB,
      if T+ and T- are unconnected they go high
    - it is strongly recommended to add a 10nF/0.01mF ceramic surface-mount
      capacitor, placed across the T+ and T- pins, to filter noise on the
      thermocouple lines
*/
/**************************************************************************/
    float MAX31855::getTemperature(int32_t rawValue) {
        if (rawValue == MAX31855_FORCE_READ_DATA) rawValue = readRawData();

        if (detectThermocouple(rawValue) != MAX31855_THERMOCOUPLE_OK) return MAX31855_ERROR;

        rawValue = rawValue >> 18; //clear D17..D0 bits

        return (float) rawValue * MAX31855_THERMOCOUPLE_RESOLUTION;
    }

/**************************************************************************/
/*
    getColdJunctionTemperature()

    Reads Temperature, C

    NOTE:
    - range -40°C..+125° ±3°C with 0.062°C resolution/increment
    - chip internal temperature data is 12-bit long
    - bit D15 is cold-junction temperature sign bit "+" is high & "-" is low
    - bits D14..D4 contain cold-junction temperature in the order of MSB to LSB
*/
/**************************************************************************/
    float MAX31855::getColdJunctionTemperature(int32_t rawValue) {
        if (rawValue == MAX31855_FORCE_READ_DATA) rawValue = readRawData();

        if (getChipID(rawValue) != MAX31855_ID) return MAX31855_ERROR;

        rawValue = rawValue & 0x0000FFFF; //clear D31..D16 bits
        rawValue = rawValue >> 4;         //clear D3...D0  bits

        return (float) rawValue * MAX31855_COLD_JUNCTION_RESOLUTION;
    }

/**************************************************************************/
/*
    readRawData()

    Reads raw data from MAX31855 via hardware SPI

    NOTE:
    - max SPI clock speed for MAX31855 is 5MHz
    - in SPI_MODE0 data available shortly after the rising edge of SCK
    - read of the cold-junction compensated thermocouple temperature requires
      14 clock cycles
    - read of the cold-junction compensated thermocouple temperature & reference
      junction temperatures requires 32 clock cycles
    - forcing CS low immediately stops any conversion process, force CS high
      to initiate a new measurement process
    - set CS low to enable the serial interface & force to output the first bit on the SO pin,
      apply 14/32 clock signals at SCK to read the results at SO on the falling edge of the SCK
    - bit D31 is the thermocouple temperature sign bit "+" is high & "-" is low,
      if T+ & T- pins are unconnected it goes low
    - bits D30..D18 contain the converted temperature in the order of MSB to LSB,
      if T+ & T- pins are unconnected they go high
    - bit D17 is low to provide a device ID for the MAX31855
    - bit D16 is normally low & goes high if thermocouple is open, shorted to GND or VCC
    - bit D15 is cold-junction temperature sign bit "+" is high & "-" is low
    - bits D14..D4 contain cold-junction temperature in the order of MSB to LSB
    - bit D3 is is low to provide a device ID for the MAX31855
    - bit D2 is normally low & goes high to indicate a hermocouple short to VCC
    - bit D1 is normally low & goes high to indicate a thermocouple short to GND
    - bit D0 is normally low & goes high to indicate a thermocouple open circuit

    - 8-bit  16MHz  AVR  one clock cycle is 62.5nS
    - 32-bit 80MHz/180 ESP8266 one clock cycle is 12.5nS/5.5nS
    - 32-bit 160MHz/250MHz ESP32 one clock cycle is 6.25nS/4nS
    - 32-bit 72MHz STM32 one clock cycle is 13.9nS
    - arduino 8-bit AVR maximum SPI master clock speed is mcu speed/2,
      for 5v-16MHz/ProMini speed is 16000000/2=8MHz 
    - arduino ESP8266 maximum SPI master clock speed is 80000000=80MHz
    - arduino STM32 maximum SPI master clock speed is mcu speed/2,
      for STM32F103C8 speed is 72000000/2=36MHz
*/
/**************************************************************************/
    int32_t MAX31855::readRawData(void) {
        int32_t rawData = 0;
        uint8_t rx_buf[4]{0, 0, 0, 0};
        uint8_t tx_buf[4]{0, 0, 0, 0};
        //uint8_t rx_buf[2] {0, 0 };
        //uint8_t tx_buf[2] {0, 0};
        mcp23017->digitalWrite(_cs, LOW);                                          //stop  measurement/conversion
        usleep(1);
        //delayMicroseconds(1);                                            //pulse fall time > 100nS
        mcp23017->digitalWrite(_cs, HIGH);                                         //start measurement/conversion

        delay(MAX31855_CONVERSION_TIME);

        //SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0)); //up to 5MHz, read MSB first, SPI mode 0, see note

        mcp23017->digitalWrite(_cs,
                               LOW);                                          //set software CS low to enable SPI interface for MAX31855

        spi_exchange(&_spi, rx_buf, tx_buf, sizeof(int32_t));
        //spi_exchange(&_spi, (void*) &rawData, tx_buf, sizeof(int32_t));
        std::reverse(rx_buf, rx_buf + 4);
        memcpy((int32_t *)&rawData, rx_buf, sizeof(int32_t));

    //    if (mcp23017->bitRead(rawData, 3) != 0)
    //        printf("cs %d, bit 3 not 0\n", _cs);

    //    if (mcp23017->bitRead(rawData, 17) != 0)
    //        printf("cs %d, bit 17 not 0\n", _cs);
        /*
        for (uint8_t i = 0; i < 2; i++)                                  //read 32-bits via hardware SPI, in order MSB->LSB (D31..D0 bit)
        {
            int16_t tmp;
          spi_exchange(&_spi, (void*) &tmp, tx_buf, sizeof(int16_t));
          rawData = (rawData << 16) | tmp;            //chip has read only SPI & MOSI not connected, so it doesn't metter what to send
        }
         */

        mcp23017->digitalWrite(_cs,
                               HIGH);                                         //disables SPI interface for MAX31855, but it will initiate measurement/conversion

        //SPI.endTransaction();                                            //de-asserting hardware CS & free hw SPI for other slaves

        //rawData &= 0xffff;
        return rawData;
    }

    void MAX31855::delay(unsigned int msec) //in milliseconds
    {
        clock_t target_value = msec * (CLOCKS_PER_SEC / 1000) + clock();
        while (target_value < (clock_t)msec - clock());
    }

    unsigned int MAX31855::reverseBits(unsigned int x, int k) {

        if (k & 1) x = ((x & 0x55555555) << 1 | (x & 0xAAAAAAAA) >> 1);

        if (k & 2) x = ((x & 0x33333333) << 2 | (x & 0xCCCCCCCC) >> 2);
        if (k & 4) x = ((x & 0x0f0f0f0f) << 4 | (x & 0xf0f0f0f0) >> 4);
        if (k & 8) x = ((x & 0x00ff00ff) << 8 | (x & 0xff00ff00) >> 8);
        if (k & 16) x = ((x & 0x0000ffff) << 16 | (x & 0xffff000) >> 16);

        return x;
    }

    void MAX31855::setCS(uint8_t cs){
        _cs = cs;
    }

    void MAX31855::addCS(uint8_t cs) {
        mcp23017->pinMode(cs, OUTPUT);
        mcp23017->digitalWrite(cs, HIGH);
    }

    void MAX31855::addCS(std::vector<uint8_t> &cs){
        for(auto pin : cs){
            addCS(pin);
        }
    }
}


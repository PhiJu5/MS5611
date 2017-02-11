#include <Arduino.h>
#include <math.h>
#if defined(CORE_TEENSY) || defined(TEENSYDUINO)
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif
#endif

const uint8_t MS5611_ADDRESS = 0x77;
const uint8_t MS5611_CMD_ADC_READ = 0x00;
const uint8_t MS5611_CMD_RESET = 0x1E;
const uint8_t MS5611_CMD_CONV_D1 = 0x40;
const uint8_t  MS5611_CMD_CONV_D2 = 0x50;
const uint8_t MS5611_CMD_READ_PROM = 0xA2;

#define MS5611_C1					(calibrationData[0])
#define MS5611_C2					(calibrationData[1])
#define MS5611_C3					(calibrationData[2])
#define MS5611_C4					(calibrationData[3])
#define MS5611_C5					(calibrationData[4])
#define MS5611_C6					(calibrationData[5])

{
#if defined(CORE_TEENSY) || defined(TEENSYDUINO)
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.2
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else // AVR
  Wire.begin();
  // Disable internal pullup, see Arduino\hardware\arduino\avr\libraries\Wire\src\utility\twi.c
  Wire.setClock(400000L);
#endif
#endif
  reset();
  setADC(adcD1, adcD2);
  readPROM();
  }
}

void MS5611::setADC(MS5611_ADC_t adcD1, MS5611_ADC_t adcD2)
{
  switch (adcD1)
  {
    case MS5611_ULTRA_LOW_POWER:
      convertionTimeD1 = MS5611_ULTRA_LOW_POWERADC_TIME;
      break;
    case MS5611_LOW_POWER:
      convertionTimeD1 = MS5611_LOW_POWERADC_TIME;
      break;
    case MS5611_STANDARD:
      convertionTimeD1 = MS5611_STANDARDADC_TIME;
      break;
    case MS5611_HIGH_RES:
      convertionTimeD1 = MS5611_HIGHADC_TIME;
      break;
    case MS5611_ULTRA_HIGH_RES:
      convertionTimeD1 = MS5611_ULTRA_HIGHADC_TIME;
      break;
  }
  adcRateD1 = adcD1;
  switch (adcD2)
  {
    case MS5611_ULTRA_LOW_POWER:
      convertionTimeD2 = MS5611_ULTRA_LOW_POWERADC_TIME;
      break;
    case MS5611_LOW_POWER:
      convertionTimeD2 = MS5611_LOW_POWERADC_TIME;
      break;
    case MS5611_STANDARD:
      convertionTimeD2 = MS5611_STANDARDADC_TIME;
      break;
    case MS5611_HIGH_RES:
      convertionTimeD2 = MS5611_HIGHADC_TIME;
      break;
    case MS5611_ULTRA_HIGH_RES:
      convertionTimeD2 = MS5611_ULTRA_HIGHADC_TIME;
      break;
  }
  adcRateD2 = adcD2;
}

void MS5611::reset()
{
  Wire.write(MS5611_CMD_RESET);
  Wire.endTransmission();
}

void MS5611::readPROM()
{
  for (uint8_t offset = 0; offset < 6; offset++)
  {
    calibrationData[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
  }
}

// Digital Pressure value
{
  Wire.write(MS5611_CMD_CONV_D1 + adcRateD1);
  Wire.endTransmission();
  return readRegister24(MS5611_CMD_ADC_READ);
}

// Digital Temperature value
{
  Wire.write(MS5611_CMD_CONV_D2 + adcRateD2);
  Wire.endTransmission();
  return readRegister24(MS5611_CMD_ADC_READ);
}

uint16_t MS5611::readRegister16(uint8_t reg)
{
  Wire.write(reg);
  Wire.endTransmission();
    uint16_t result = Wire.read();
    result <<= 8 ;
    result |= Wire.read();
    return result;
  } else {
    // failure ... maybe slave wasn't ready or not connected
    return 0;
  }
}

uint32_t MS5611::readRegister24(uint8_t reg)
{
  Wire.write(reg);
  Wire.endTransmission();
    uint32_t result = Wire.read();
    result <<= 8 ;
    result |= Wire.read();
    result <<= 8 ;
    result |= Wire.read();
    return result;
  } else {
    // failure ... maybe slave wasn't ready or not connected
    return 0;
  }
}

{
  referencePressure = refPressure;
}

{
  return referencePressure;
}

// Calculate altitude from Pressure and a Reference pressure
{
}

// Calculate sea level from Pressure given on specific altitude
{
}

int32_t MS5611::computePressure(int32_t D1, int32_t D2, volatile int32_t *temperature)
{
  int64_t dT = (int64_t)D2 - ((int64_t)MS5611_C5 << 8);
  int64_t OFF = ((int64_t)MS5611_C2 << 16) + (((int64_t)MS5611_C4 * dT) >> 7);
  int64_t SENS = ((int64_t)MS5611_C1 << 15) + (((int64_t)MS5611_C3 * dT) >> 8);
  int32_t TEMP = (int32_t)(2000 + ((((int64_t)dT * (int64_t)MS5611_C6)) >> 23));
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;
  int32_t TEMP2 = 0;
  if (TEMP < 2000)
  {
    TEMP2 = (int32_t)((dT * dT) >> 31);
    int64_t tmp = (int64_t)TEMP - 2000;
    tmp *= tmp;
    OFF2 = 5 * (tmp >> 1);
    SENS2 = 5 * (tmp >> 2);
  }
  if (TEMP < -1500)
  {
    int64_t tmp = (int64_t)TEMP + 1500;
    tmp *= tmp;
    OFF2 = OFF2 + 7 * tmp;
    SENS2 = SENS2 + (11 * (tmp >> 1));
  }
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;
  TEMP = TEMP - TEMP2;
  *temperature = TEMP;
  return (int32_t)(((((int64_t)D1 * (int64_t)SENS) >> 21) - OFF) >> 15);
}

void MS5611::askD1()
{
  Wire.write(MS5611_CMD_CONV_D1 + adcRateD1);
  Wire.endTransmission();
}

void MS5611::askD2()
{
  Wire.write(MS5611_CMD_CONV_D2 + adcRateD2);
  Wire.endTransmission();
}

uint32_t MS5611::readAdc()
{
  return readRegister24(MS5611_CMD_ADC_READ);
}



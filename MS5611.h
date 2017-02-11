#ifndef MS5611_h
#define MS5611_h

#include <Arduino.h>

class MS5611
{
  public:
    typedef enum
    {
      MS5611_ULTRA_HIGH_RES   = 0x08, // ACD time 9.04 milli seconds
      MS5611_HIGH_RES         = 0x06, // ACD time 4.54 milli seconds
      MS5611_STANDARD         = 0x04, // ACD time 2.28 milli seconds
      MS5611_LOW_POWER        = 0x02, // ACD time 1.17 milli seconds
      MS5611_ULTRA_LOW_POWER  = 0x00  // ACD time 0.60 milli seconds
    } MS5611_ADC_t;
    double getSeaPressure() {
      return 101325.00d;
    }
    void begin(MS5611_ADC_t adcD1, MS5611_ADC_t adcD2);
    int32_t readTemperature(boolean secondOrderTemperatureCompensation);
    int32_t readPressure(boolean secondOrderTemperatureCompensation);
    void setReferencePressure(double refPressure);
    double getReferencePressure();
    double getAltitude(double pressure, double pressureRef);
    double getSeaLevel(double pressure, double altitude);
    int32_t computePressure(int32_t D1, int32_t D2, volatile int32_t *temperature);
    void askD1();
    void askD2();
    uint32_t readAdc();
  private:;
    typedef enum
    {
      // in micros seconds
      MS5611_ULTRA_HIGHADC_TIME      = 10, // 9.04 milli seconds
      MS5611_HIGHADC_TIME            = 5,  // 4.54 milli seconds
      MS5611_STANDARDADC_TIME        = 3,  // 2.28 milli seconds
      MS5611_LOW_POWERADC_TIME       = 2,  // 1.17 milli seconds
      MS5611_ULTRA_LOW_POWERADC_TIME = 1   // 0.6 milli seconds
    } MS5611_ADCADC_TIME_t;
    uint16_t calibrationData[6];
    unsigned int convertionTimeD1; // micro seconds
    unsigned int convertionTimeD2; // microsseconds
    uint8_t adcRateD1;
    uint8_t adcRateD2;
    double referencePressure;
    void reset(void);
    void readPROM(void);
    MS5611_ADC_t getADC();
    void setADC(MS5611_ADC_t adcD1, MS5611_ADC_t adcD2);
    int32_t readD1(); // Digital Pressure value
    int32_t readD2(); // Digital Temperature value
    uint16_t readRegister16(uint8_t reg);
    uint32_t readRegister24(uint8_t reg);
};

#endif

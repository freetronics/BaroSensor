/* BaroSensor Arduino library, for Freetronics BARO module (MS5637-02BA03)
 * http://www.freetronics.com/baro
 *
 * Copyright (C)2014 Freetronics Pty Ltd. Licensed under GNU GPLv3 as described in the LICENSE file.
 *
 * Written by Angus Gratton (angus at freetronics dot com)
 */
#include "BaroSensor.h"

/* i2c address of module */
#define BARO_ADDR 0x76

/* delay to wait for sampling to complete, on each OSR level */
const uint8_t SamplingDelayMs[6] PROGMEM = {
  1,
  2,
  3,
  5,
  9,
  17
};

/* module commands */
#define CMD_RESET 0x1E
#define CMD_PROM_READ(offs) (0xA0+(offs<<1)) /* Offset 0-7 */
#define CMD_START_D1(oversample_level) (0x40 + 2*(int)oversample_level)
#define CMD_START_D2(oversample_level) (0x50 + 2*(int)oversample_level)
#define CMD_READ_ADC 0x00

BaroSensorClass BaroSensor;

void BaroSensorClass::begin()
{
  Wire.begin();
  Wire.beginTransmission(BARO_ADDR);
  Wire.write(CMD_RESET);
  err = Wire.endTransmission();
  if(err) return;

  uint16_t prom[7];
  for(int i = 0; i < 7; i++) {
    Wire.beginTransmission(BARO_ADDR);
    Wire.write(CMD_PROM_READ(i));
    err = Wire.endTransmission(false);
    if(err)
      return;
    if(Wire.requestFrom(BARO_ADDR, 2) != 2) {
      err = -2;
      return;
    }
    prom[i] = ((uint16_t)Wire.read()) << 8;
    prom[i] |= Wire.read();
  }

  // TODO: verify CRC4 stored in high nibble of prom[0]

  c1 = prom[1];
  c2 = prom[2];
  c3 = prom[3];
  c4 = prom[4];
  c5 = prom[5];
  c6 = prom[6];
}

float BaroSensorClass::getTemperature(TempUnit scale, BaroOversampleLevel level)
{
  float result;
  if(getTempAndPressure(&result, NULL, scale, level))
    return result;
  else
    return NAN;
}

float BaroSensorClass::getPressure(BaroOversampleLevel level)
{
  float result;
  if(getTempAndPressure(NULL, &result, CELSIUS, level))
    return result;
  else
    return NAN;
}

bool BaroSensorClass::getTempAndPressure(float *temperature, float *pressure, TempUnit tempScale, BaroOversampleLevel level)
{
  if(err)
    return false;

  uint32_t d2 = takeReading(CMD_START_D2(level), level);
  if(d2 == 0)
    return false;
  int32_t dt = d2 - c5 * (1L<<8);

  if(temperature != NULL) {
    int32_t temp = 2000 + (int64_t)dt * c6 / (1L<<23);
    *temperature = (float)temp / 100;
    if(tempScale == FAHRENHEIT)
      *temperature = *temperature * 9 / 5 + 32;
  }

  if(pressure != NULL) {
    uint32_t d1 = takeReading(CMD_START_D1(level), level);
    if(d1 == 0)
      return false;

    int64_t off = c2 * (1LL<<17) + (c4 * dt) / (1LL<<6);
    int64_t sens = c1 * (1LL<<16) + (c3 * dt) / (1LL<<7);
    int32_t p = (d1 * sens/(1LL<<21) - off) / (1LL << 15);
    *pressure = (float)p / 100;
  }
  return true;
}

uint32_t BaroSensorClass::takeReading(uint8_t trigger_cmd, BaroOversampleLevel oversample_level)
{
  Wire.beginTransmission(BARO_ADDR);
  Wire.write(trigger_cmd);
  err = Wire.endTransmission();
  if(err)
    return 0;
  uint8_t sampling_delay = pgm_read_byte(SamplingDelayMs + (int)oversample_level);
  delay(sampling_delay);

  Wire.beginTransmission(BARO_ADDR);
  Wire.write(CMD_READ_ADC);
  err = Wire.endTransmission(false);
  if(err)
    return 0;
  if(Wire.requestFrom(BARO_ADDR, 3) != 3) {
    err = -2;
    return 0;
  }
  uint32_t result = (uint32_t)Wire.read() << 16;
  result |= (uint32_t)Wire.read() << 8;
  result |= Wire.read();
  return result;
}


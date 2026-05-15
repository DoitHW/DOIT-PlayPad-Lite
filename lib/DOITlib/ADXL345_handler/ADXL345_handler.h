#pragma once

#include <Adafruit_ADXL345_U.h>
#include <defines_DMS/defines_DMS.h>
#include <Wire.h>

#if defined(DOIT_LITE) && defined(DEVKIT)
  #define SDA_ADXL_PIN 42
  #define SCL_ADXL_PIN 2
  #define INT1_ADXL_PIN 39
  #define INT2_ADXL_PIN 40
#else
  #define SDA_ADXL_PIN 47
  #define SCL_ADXL_PIN 48
  #define INT1_ADXL_PIN 14
  #define INT2_ADXL_PIN 15
#endif

class ADXL345Handler {
public:
  ADXL345Handler();
  void init();
  void enableActivityInterrupt(uint16_t thresholdMg, bool enX, bool enY,
                               bool enZ);
  void clearInterrupts();
  bool isInitialized() const { return initialized_; }

private:
  Adafruit_ADXL345_Unified accel_;
  bool initialized_ = false;
};

extern ADXL345Handler adxl345Handler;

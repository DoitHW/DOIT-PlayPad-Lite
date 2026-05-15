#include <ADXL345_handler/ADXL345_handler.h>

#include <Arduino.h>

namespace {

constexpr uint8_t kAdxlAddress = 0x53;
constexpr uint8_t ADXL_REG_THRESH_ACT = 0x24;
constexpr uint8_t ADXL_REG_ACT_INACT_CTL = 0x27;
constexpr uint8_t ADXL_REG_BW_RATE = 0x2C;
constexpr uint8_t ADXL_REG_POWER_CTL = 0x2D;
constexpr uint8_t ADXL_REG_INT_ENABLE = 0x2E;
constexpr uint8_t ADXL_REG_INT_MAP = 0x2F;
constexpr uint8_t ADXL_REG_INT_SOURCE = 0x30;
constexpr uint8_t ADXL_REG_DATA_FORMAT = 0x31;

void adxlWrite8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAdxlAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t adxlRead8(uint8_t reg) {
  Wire.beginTransmission(kAdxlAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(kAdxlAddress, static_cast<uint8_t>(1));
  return Wire.available() ? Wire.read() : 0;
}

} // namespace

ADXL345Handler adxl345Handler;

ADXL345Handler::ADXL345Handler() : accel_(Adafruit_ADXL345_Unified(12345)) {}

void ADXL345Handler::init() {
  if (initialized_) {
    Wire.end();
  }

  Wire.begin(SDA_ADXL_PIN, SCL_ADXL_PIN);
  Wire.setClock(100000);

  bool detected = false;
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (accel_.begin()) {
      detected = true;
      break;
    }

    Wire.end();
    delay(50);
    Wire.begin(SDA_ADXL_PIN, SCL_ADXL_PIN);
    delay(100);
  }

  initialized_ = detected;
  if (!initialized_) {
    return;
  }

  accel_.setRange(ADXL345_RANGE_2_G);
}

void ADXL345Handler::enableActivityInterrupt(uint16_t thresholdMg, bool enX,
                                             bool enY, bool enZ) {
  if (!initialized_) {
    init();
  }
  if (!initialized_) {
    return;
  }

  uint8_t format = adxlRead8(ADXL_REG_DATA_FORMAT);
  format = (format & ~0x03) | 0x03;
  adxlWrite8(ADXL_REG_DATA_FORMAT, format);
  adxlWrite8(ADXL_REG_BW_RATE, 0x08);

  uint16_t thresholdLsbs = static_cast<uint16_t>(lroundf(thresholdMg / 62.5f));
  thresholdLsbs = constrain(thresholdLsbs, static_cast<uint16_t>(1),
                            static_cast<uint16_t>(255));
  adxlWrite8(ADXL_REG_THRESH_ACT, static_cast<uint8_t>(thresholdLsbs));

  uint8_t activityConfig = 1 << 7;
  if (enX) activityConfig |= 1 << 6;
  if (enY) activityConfig |= 1 << 5;
  if (enZ) activityConfig |= 1 << 4;
  adxlWrite8(ADXL_REG_ACT_INACT_CTL, activityConfig);

  uint8_t intMap = adxlRead8(ADXL_REG_INT_MAP);
  intMap &= ~(1 << 4);
  adxlWrite8(ADXL_REG_INT_MAP, intMap);

  uint8_t intEnable = adxlRead8(ADXL_REG_INT_ENABLE);
  intEnable |= 1 << 4;
  adxlWrite8(ADXL_REG_INT_ENABLE, intEnable);
  adxlWrite8(ADXL_REG_POWER_CTL, 0x08);
}

void ADXL345Handler::clearInterrupts() {
  (void)adxlRead8(ADXL_REG_INT_SOURCE);
}

#include <Arduino.h>
#include <FastLED.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <ADXL345_handler/ADXL345_handler.h>
#include <CommunicatorLite/CommunicatorLite.h>
#include <Frame_DMS/Frame_DMS.h>
#include <LiteStorage/LiteStorage.h>
#include <RoomScannerLite/RoomScannerLite.h>
#include <defines_DMS/defines_DMS.h>

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kSleepAfterMs = 5UL * 60UL * 1000UL;
constexpr uint8_t kLedBrightness = 96;
constexpr uint8_t kFirmwareVersion = 0x01;
constexpr uint8_t kBuildMonth = 0x01;

uint16_t SERIAL_NUM =
    (static_cast<uint16_t>(kBuildMonth) << 8) | kFirmwareVersion;

unsigned long lastDisplayInteraction = 0;
SemaphoreHandle_t uartTxMutex = nullptr;
bool adxl = true;

extern std::vector<uint8_t> printTargetID;

CRGB leds[NUM_LEDS];
CommunicatorLite communicator;
RoomScannerLite scanner;

namespace {

uint32_t lastActivityMs = 0;
bool buttonStablePressed = false;
bool buttonLastReading = false;
uint32_t buttonLastChangeMs = 0;
bool adxlInt1LastState = false;
bool relayState = false;

void initDebugSerial() {
  Serial.begin(kSerialBaud);
#if ARDUINO_USB_CDC_ON_BOOT
  const uint32_t waitStartMs = millis();
  while (!Serial && (millis() - waitStartMs < 3000)) {
    delay(10);
  }
#else
  delay(100);
#endif
  Serial.println();
  Serial.println("DOIT PlayPad Lite boot");
  Serial.printf("Serial monitor ready @ %lu ms\n", millis());
}

CRGB colorFromHex(uint32_t rgb) {
  return CRGB((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
}

const CRGB kButtonPalette[NUM_LEDS] = {
    colorFromHex(0xFF2300), // relay
    colorFromHex(0xFFFFAA), // blanco
    colorFromHex(0xFF0000), // rojo
    colorFromHex(0x00FFC8), // celeste
    colorFromHex(0xFF9B00), // amarillo
    colorFromHex(0xFF5900), // naranja
    colorFromHex(0x00FF00), // verde
    colorFromHex(0xFF00D2), // violeta
    colorFromHex(0x0000FF), // azul
};

constexpr uint8_t kDirectButtonPins[NUM_LEDS] = {
    1,  // relay
    2,  // blanco
    42, // rojo
    41, // celeste
    40, // amarillo
    39, // naranja
    38, // verde
    36, // morado
    37, // azul
};

const char *const kDirectButtonNames[NUM_LEDS] = {
    "RELAY", "BLANCO", "ROJO", "CELESTE", "AMARILLO",
    "NARANJA", "VERDE", "MORADO", "AZUL",
};

constexpr uint8_t kDirectButtonColorCodes[NUM_LEDS] = {
    RELAY, WHITE, RED, LIGHT_BLUE, YELLOW, ORANGE, GREEN, VIOLET, BLUE,
};

void markActivity() {
  lastActivityMs = millis();
  lastDisplayInteraction = lastActivityMs;
}

CRGB relayIdleColor() {
  const uint8_t amount = beatsin8(14, 0, 255);
  return blend(CRGB::Blue, CRGB::Cyan, amount);
}

void showDefaultButtons() {
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    leds[i] = kButtonPalette[i];
  }
  leds[0] = relayIdleColor();
  FastLED.show();
}

void updateIdleLedAnimation() {
  static uint32_t lastRefreshMs = 0;
  const uint32_t now = millis();
  if (now - lastRefreshMs < 25) {
    return;
  }

  lastRefreshMs = now;
  leds[0] = relayIdleColor();
  FastLED.show();
}

void flashAll(const CRGB &color, uint8_t times = 2, uint16_t holdMs = 90) {
  for (uint8_t i = 0; i < times; ++i) {
    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
    delay(holdMs);
    showDefaultButtons();
    delay(holdMs);
  }
}

void welcomeEffect() {
  FastLED.clear(true);
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    leds[i] = kButtonPalette[i];
    FastLED.show();
    delay(55);
  }

  for (uint8_t pass = 0; pass < 2; ++pass) {
    for (uint8_t level = 40; level <= kLedBrightness; level += 8) {
      FastLED.setBrightness(level);
      FastLED.show();
      delay(18);
    }
    for (int level = kLedBrightness; level >= 40; level -= 8) {
      FastLED.setBrightness(level);
      FastLED.show();
      delay(18);
    }
  }

  FastLED.setBrightness(kLedBrightness);
  showDefaultButtons();
}

void scanningTick() {
  static uint8_t hue = 140;
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CHSV(hue + (i * 8), 180, 120);
  }
  hue += 3;
  FastLED.show();
}

void scanFeedback(LiteScanEvent event, const TARGETNS *) {
  switch (event) {
  case LiteScanEvent::Started:
    Serial.println("[SCAN] started");
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();
    break;
  case LiteScanEvent::Tick:
    Serial.println("[SCAN] tick");
    scanningTick();
    break;
  case LiteScanEvent::Discovered:
    Serial.println("[SCAN] target discovered");
    scanningTick();
    break;
  case LiteScanEvent::Saved:
    Serial.println("[SCAN] target saved");
    flashAll(CRGB::Green, 1);
    break;
  case LiteScanEvent::Duplicate:
    Serial.println("[SCAN] target duplicate");
    flashAll(CRGB::Yellow, 1);
    break;
  case LiteScanEvent::SaveFailed:
    Serial.println("[SCAN] target save failed");
    flashAll(CRGB::Red, 2);
    break;
  case LiteScanEvent::Finished:
    Serial.println("[SCAN] finished");
    showDefaultButtons();
    break;
  }
}

TARGETNS calculateOwnNS() {
  String macNumbers = WiFi.macAddress();
  macNumbers = macNumbers.substring(9);
  macNumbers.replace(":", "");

  char serialPrefix[5];
  snprintf(serialPrefix, sizeof(serialPrefix), "%04X", SERIAL_NUM);

  return TARGETNS{
      static_cast<byte>(strtoul(String(serialPrefix).substring(0, 2).c_str(),
                                nullptr, 16)),
      static_cast<byte>(strtoul(String(serialPrefix).substring(2, 4).c_str(),
                                nullptr, 16)),
      static_cast<byte>(strtoul(macNumbers.substring(0, 2).c_str(), nullptr, 16)),
      static_cast<byte>(strtoul(macNumbers.substring(2, 4).c_str(), nullptr, 16)),
      static_cast<byte>(strtoul(macNumbers.substring(4, 6).c_str(), nullptr, 16)),
  };
}

void printNS(const char *label, const TARGETNS &ns) {
  Serial.printf("%s%02X%02X%02X%02X%02X\n", label, ns.mac01, ns.mac02,
                ns.mac03, ns.mac04, ns.mac05);
}

void printTarget(const char *label, uint8_t targetType, const TARGETNS &targetNS) {
  Serial.printf("%s targetType=0x%02X targetNS=%02X%02X%02X%02X%02X\n", label,
                targetType, targetNS.mac01, targetNS.mac02, targetNS.mac03,
                targetNS.mac04, targetNS.mac05);
}

void sendButtonFrame(uint8_t buttonIndex) {
  const uint8_t targetType = communicator.activeTargetType();
  const TARGETNS targetNS = communicator.activeTargetNS();

  if (buttonIndex == 0) {
    relayState = !relayState;
    Serial.printf("[INPUT] RELAY send F_SEND_FLAG_BYTE value=0x%02X\n",
                  relayState ? 0x01 : 0x00);
    printTarget("[INPUT] RELAY", targetType, targetNS);
    send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, targetType, targetNS,
                                         relayState ? 0x01 : 0x00));
    return;
  }

  const uint8_t color = kDirectButtonColorCodes[buttonIndex];
  Serial.printf("[INPUT] %s send F_SEND_COLOR color=0x%02X\n",
                kDirectButtonNames[buttonIndex], color);
  printTarget("[INPUT] COLOR", targetType, targetNS);
  send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, targetType, targetNS,
                                   color));
}

void initDirectButtons() {
  pinMode(ENC_BUTTON, INPUT_PULLUP);
  for (uint8_t pin : kDirectButtonPins) {
    pinMode(pin, INPUT_PULLUP);
  }

  Serial.printf("[INPUT] SIDE pin GPIO%u initial=%s\n", ENC_BUTTON,
                digitalRead(ENC_BUTTON) == LOW ? "PRESSED" : "released");
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    Serial.printf("[INPUT] BTN[%u] %-8s GPIO%u initial=%s\n", i,
                  kDirectButtonNames[i], kDirectButtonPins[i],
                  digitalRead(kDirectButtonPins[i]) == LOW ? "PRESSED"
                                                           : "released");
  }
}

void pollDirectButtonsForActivity() {
  static bool lastRawPressed[NUM_LEDS] = {false};
  static bool pressArmed[NUM_LEDS] = {true, true, true, true, true,
                                      true, true, true, true};
  static uint32_t lastRawChangeMs[NUM_LEDS] = {0};
  static uint32_t lastPressSentMs[NUM_LEDS] = {0};
  constexpr uint32_t kDebounceMs = 12;
  constexpr uint32_t kMinRepeatGapMs = 90;

  const uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    const bool rawPressed = (digitalRead(kDirectButtonPins[i]) == LOW);

    if (rawPressed != lastRawPressed[i]) {
      lastRawPressed[i] = rawPressed;
      lastRawChangeMs[i] = now;
      Serial.printf("[INPUT RAW] BTN[%u] %-8s GPIO%u %s @ %lu ms\n", i,
                    kDirectButtonNames[i], kDirectButtonPins[i],
                    rawPressed ? "LOW/PRESS" : "HIGH/RELEASE", now);
    }

    if (now - lastRawChangeMs[i] < kDebounceMs) {
      continue;
    }

    if (!rawPressed) {
      pressArmed[i] = true;
      continue;
    }

    if (pressArmed[i] && (now - lastPressSentMs[i] >= kMinRepeatGapMs)) {
      pressArmed[i] = false;
      lastPressSentMs[i] = now;
      Serial.printf("[INPUT] BTN[%u] %-8s GPIO%u %s @ %lu ms\n", i,
                    kDirectButtonNames[i], kDirectButtonPins[i], "PRESS", now);
      markActivity();
      sendButtonFrame(i);
    }
  }
}

void initRF() {
  Serial1.end();
  delay(10);
  Serial1.begin(RF_BAUD_RATE, SERIAL_8N1, RF_RX_PIN, RF_TX_PIN);
  while (Serial1.available()) {
    (void)Serial1.read();
  }
  uartBuffer.clear();
  uartBuffer.reserve(MAX_BUFFER_SIZE);
  printTargetID.reserve(5);
  Serial1.onReceive(onUartInterrupt);
  Serial.printf("[RF] Serial1 begin baud=%lu rxGPIO=%u txGPIO=%u\n",
                static_cast<unsigned long>(RF_BAUD_RATE), RF_RX_PIN, RF_TX_PIN);
}

void initOwnNS() {
  const TARGETNS ownNS = calculateOwnNS();
  setLocalNS(ownNS);
  setLocalRoom(DEFAULT_ROOM);
  printNS("PlayPad Lite NS: ", ownNS);
  WiFi.mode(WIFI_OFF);
}

void initAdxl() {
#ifdef ADXL
  adxl345Handler.init();
  pinMode(INT1_ADXL_PIN, INPUT_PULLDOWN);
  if (adxl345Handler.isInitialized()) {
    adxl345Handler.enableActivityInterrupt(600, true, true, true);
    adxl345Handler.clearInterrupts();
    adxlInt1LastState = (digitalRead(INT1_ADXL_PIN) == HIGH);
    Serial.printf("[ADXL] initialized sdaGPIO=%u sclGPIO=%u int1GPIO=%u int2GPIO=%u int1=%s\n",
                  SDA_ADXL_PIN, SCL_ADXL_PIN, INT1_ADXL_PIN, INT2_ADXL_PIN,
                  adxlInt1LastState ? "HIGH" : "low");
  } else {
    Serial.printf("[ADXL] init failed sdaGPIO=%u sclGPIO=%u\n", SDA_ADXL_PIN,
                  SCL_ADXL_PIN);
  }
#endif
}

void processIncomingFrame() {
  if (!frameReceived) {
    return;
  }

  frameReceived = false;
  const LAST_ENTRY_FRAME_T frame = extract_info_from_frameIn(uartBuffer);
  Serial.printf("[RF RX] room=0x%02X origin=0x%02X originNS=%02X%02X%02X%02X%02X targetType=0x%02X function=0x%02X dataLen=%u\n",
                frame.room, frame.origin, frame.originNS.mac01,
                frame.originNS.mac02, frame.originNS.mac03,
                frame.originNS.mac04, frame.originNS.mac05, frame.targetType,
                frame.function, static_cast<unsigned>(frame.data.size()));
  markActivity();
}

void waitButtonRelease() {
  while (digitalRead(ENC_BUTTON) == LOW) {
    delay(10);
  }
  buttonStablePressed = false;
  buttonLastReading = false;
  buttonLastChangeMs = millis();
  Serial.printf("[INPUT] SIDE GPIO%u RELEASE @ %lu ms\n", ENC_BUTTON, millis());
}

bool cycleButtonPressed() {
  static bool lastRawPressed = false;
  static bool pressArmed = true;
  static uint32_t lastRawChangeMs = 0;
  static uint32_t lastPressSentMs = 0;
  constexpr uint32_t kDebounceMs = 15;
  constexpr uint32_t kMinRepeatGapMs = 160;

  const bool rawPressed = (digitalRead(ENC_BUTTON) == LOW);
  const uint32_t now = millis();

  if (rawPressed != lastRawPressed) {
    lastRawPressed = rawPressed;
    lastRawChangeMs = now;
    Serial.printf("[INPUT RAW] SIDE GPIO%u %s @ %lu ms\n", ENC_BUTTON,
                  rawPressed ? "LOW/PRESS" : "HIGH/RELEASE", now);
  }

  if (now - lastRawChangeMs < kDebounceMs) {
    return false;
  }

  if (!rawPressed) {
    pressArmed = true;
    buttonStablePressed = false;
    return false;
  }

  if (pressArmed && (now - lastPressSentMs >= kMinRepeatGapMs)) {
    pressArmed = false;
    buttonStablePressed = true;
    lastPressSentMs = now;
    Serial.printf("[INPUT] SIDE GPIO%u PRESS @ %lu ms\n", ENC_BUTTON, now);
    markActivity();
    return true;
  }

  return false;
}

void runScanAtBoot() {
  Serial.println("ENC_BUTTON held at boot: scanning room.");
  const LiteScanResult result = scanner.scan(scanFeedback);
  communicator.reloadTargets();
  Serial.printf("Scan finished. discovered=%u saved=%u duplicates=%u failed=%u\n",
                result.discovered, result.saved, result.duplicates, result.failed);

  if (result.failed > 0) {
    flashAll(CRGB::Red, 3);
  } else if (result.saved > 0) {
    flashAll(CRGB::Green, 3);
  } else {
    flashAll(CRGB::Yellow, 2);
  }

  waitButtonRelease();
}

void enterLightSleepIfIdle() {
  if (millis() - lastActivityMs < kSleepAfterMs) {
    return;
  }

#ifdef ADXL
  if (!adxl345Handler.isInitialized()) {
    return;
  }
#else
  return;
#endif

  Serial.println("Entering light sleep.");
  FastLED.clear(true);

#ifdef ADXL
  adxl345Handler.clearInterrupts();
  pinMode(INT1_ADXL_PIN, INPUT_PULLDOWN);
  gpio_wakeup_enable(static_cast<gpio_num_t>(INT1_ADXL_PIN),
                     GPIO_INTR_HIGH_LEVEL);
#endif

  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();

#ifdef ADXL
  gpio_wakeup_disable(static_cast<gpio_num_t>(INT1_ADXL_PIN));
  if (adxl345Handler.isInitialized()) {
    adxl345Handler.clearInterrupts();
  }
#endif

  Serial.println("Woke from light sleep.");
  Serial.printf("[SLEEP] wake cause=%d adxlInt1=%s\n",
                static_cast<int>(esp_sleep_get_wakeup_cause()),
                digitalRead(INT1_ADXL_PIN) == HIGH ? "HIGH" : "low");
  markActivity();
  showDefaultButtons();
}

void pollAdxlDebug() {
#ifdef ADXL
  const bool current = (digitalRead(INT1_ADXL_PIN) == HIGH);
  if (current != adxlInt1LastState) {
    adxlInt1LastState = current;
    Serial.printf("[ADXL] INT1 GPIO%u %s @ %lu ms\n", INT1_ADXL_PIN,
                  current ? "HIGH" : "low", millis());
    markActivity();
  }
#endif
}

} // namespace

void setup() {
  initDebugSerial();

  initDirectButtons();
  const bool scanRequested = (digitalRead(ENC_BUTTON) == LOW);
  Serial.printf("[BOOT] scanRequested=%s\n", scanRequested ? "yes" : "no");

  FastLED.addLeds<WS2812, BOTONERA_DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(kLedBrightness);
  welcomeEffect();

  if (!LiteStorage::begin()) {
    Serial.println("SPIFFS mount failed.");
    flashAll(CRGB::Red, 4);
  }

  initRF();
  initOwnNS();
  initAdxl();

  communicator.reloadTargets();
  Serial.printf("Loaded targets: %u\n",
                static_cast<unsigned>(LiteStorage::loadTargets().size()));

  if (scanRequested) {
    runScanAtBoot();
  }

  markActivity();
  showDefaultButtons();
}

void loop() {
  processIncomingFrame();
  pollDirectButtonsForActivity();

  if (cycleButtonPressed()) {
    Serial.println("[COMM] cycle requested");
    if (communicator.next()) {
      Serial.println("[COMM] cycle frame sequence sent");
      showDefaultButtons();
    } else {
      Serial.println("No scanned targets available for communicator cycle.");
      flashAll(CRGB::Red, 2);
    }
  }

  pollAdxlDebug();
  enterLightSleepIfIdle();
  updateIdleLedAnimation();
  delay(5);
}

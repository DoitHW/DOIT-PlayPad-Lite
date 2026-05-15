#pragma once

#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>

enum class LiteScanEvent : uint8_t {
  Started,
  Tick,
  Discovered,
  Saved,
  SaveFailed,
  Finished
};

using LiteScanFeedback = void (*)(LiteScanEvent event, const TARGETNS *ns);

struct LiteScanResult {
  uint16_t discovered = 0;
  uint16_t saved = 0;
  uint16_t failed = 0;
};

class RoomScannerLite {
public:
  LiteScanResult scan(LiteScanFeedback feedback = nullptr);
};

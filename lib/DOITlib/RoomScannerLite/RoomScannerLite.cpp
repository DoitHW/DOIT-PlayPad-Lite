#include <RoomScannerLite/RoomScannerLite.h>

#include <LiteStorage/LiteStorage.h>
#include <algorithm>
#include <vector>

namespace {

constexpr uint32_t kDiscoveryWindowMs = 30000UL;
constexpr uint32_t kPollDelayMs = 5UL;

bool containsNS(const std::vector<TARGETNS> &targets, const TARGETNS &ns) {
  return std::find_if(targets.begin(), targets.end(), [&](const TARGETNS &item) {
           return item.mac01 == ns.mac01 && item.mac02 == ns.mac02 &&
                  item.mac03 == ns.mac03 && item.mac04 == ns.mac04 &&
                  item.mac05 == ns.mac05;
         }) != targets.end();
}

bool nsIsZero(const TARGETNS &ns) {
  return (ns.mac01 | ns.mac02 | ns.mac03 | ns.mac04 | ns.mac05) == 0;
}

bool nsFromSerialPayload(const LAST_ENTRY_FRAME_T &frame, TARGETNS &out) {
  out = NS_ZERO;
  if (frame.function != F_RETURN_ELEM_SECTOR || frame.data.empty()) {
    return false;
  }
  if (frame.data[0] != ELEM_SERIAL_SECTOR) {
    return false;
  }

  if (frame.data.size() >= 6) {
    out = TARGETNS{frame.data[1], frame.data[2], frame.data[3], frame.data[4],
                   frame.data[5]};
  } else if (!nsIsZero(frame.originNS)) {
    out = frame.originNS;
  }

  return !nsIsZero(out);
}

void notify(LiteScanFeedback feedback, LiteScanEvent event, const TARGETNS *ns) {
  if (feedback) {
    feedback(event, ns);
  }
}

} // namespace

LiteScanResult RoomScannerLite::scan(LiteScanFeedback feedback) {
  LiteScanResult result;
  std::vector<TARGETNS> discovered;

  notify(feedback, LiteScanEvent::Started, nullptr);
  LiteStorage::clearTargets();

  frameReceived = false;
  uartBuffer.clear();
  send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, BROADCAST, NS_ZERO,
                                     SLEEP_SERIAL_WAKEUP_CMD));

  const uint32_t startMs = millis();
  uint32_t lastTickMs = startMs;

  while (millis() - startMs < kDiscoveryWindowMs) {
    if (frameReceived) {
      frameReceived = false;
      const LAST_ENTRY_FRAME_T frame = extract_info_from_frameIn(uartBuffer);

      TARGETNS ns = NS_ZERO;
      if (nsFromSerialPayload(frame, ns) && !containsNS(discovered, ns)) {
        discovered.push_back(ns);
        result.discovered++;
        notify(feedback, LiteScanEvent::Discovered, &ns);
      }
    }

    const uint32_t now = millis();
    if (now - lastTickMs >= 100UL) {
      notify(feedback, LiteScanEvent::Tick, nullptr);
      lastTickMs = now;
    }
    delay(kPollDelayMs);
  }

  for (const TARGETNS &ns : discovered) {
    if (LiteStorage::saveMinimalTarget(ns)) {
      result.saved++;
      notify(feedback, LiteScanEvent::Saved, &ns);
    } else {
      result.failed++;
      notify(feedback, LiteScanEvent::SaveFailed, &ns);
    }
  }

  notify(feedback, LiteScanEvent::Finished, nullptr);
  return result;
}

#include <CommunicatorLite/CommunicatorLite.h>

#include <LiteStorage/LiteStorage.h>

namespace {
constexpr uint16_t kFrameGapMs = 100;
}

void CommunicatorLite::setTargets(const std::vector<TARGETNS> &targets) {
  targets_ = targets;
  currentIndex_ = -1;
  cycleState_ = CycleState::Off;
}

void CommunicatorLite::reloadTargets() {
  setTargets(LiteStorage::loadTargets());
}

void CommunicatorLite::sendBlackout(uint8_t targetType,
                                    const TARGETNS &targetNS) {
  send_frame(
      frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, targetType, targetNS, BLACKOUT));
  delay(kFrameGapMs);
}

void CommunicatorLite::sendStart(uint8_t targetType, const TARGETNS &targetNS) {
  send_frame(
      frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, targetType, targetNS, START_CMD));
  delay(kFrameGapMs);
}

void CommunicatorLite::sendPassiveAmbient() {
  send_frame(
      frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, START_CMD));
  delay(kFrameGapMs);
  send_frame(
      frameMaker_SEND_PATTERN_NUM(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, 0x09));
  delay(kFrameGapMs);
  currentIndex_ = -1;
  cycleState_ = CycleState::PassiveAmbient;
}

uint8_t CommunicatorLite::activeTargetType() const {
  if (cycleState_ == CycleState::TargetStarted && currentIndex_ >= 0 &&
      currentIndex_ < static_cast<int>(targets_.size())) {
    return DEFAULT_DEVICE;
  }
  return BROADCAST;
}

TARGETNS CommunicatorLite::activeTargetNS() const {
  if (cycleState_ == CycleState::TargetStarted && currentIndex_ >= 0 &&
      currentIndex_ < static_cast<int>(targets_.size())) {
    return targets_[currentIndex_];
  }
  return NS_ZERO;
}

bool CommunicatorLite::next() {
  if (targets_.empty()) {
    switch (cycleState_) {
    case CycleState::Off:
    case CycleState::TargetStarted:
      sendStart(BROADCAST, NS_ZERO);
      cycleState_ = CycleState::BroadcastStarted;
      currentIndex_ = -1;
      return true;

    case CycleState::BroadcastStarted:
      sendBlackout(BROADCAST, NS_ZERO);
      cycleState_ = CycleState::Off;
      currentIndex_ = -1;
      return true;

    case CycleState::PassiveAmbient:
      sendBlackout(BROADCAST, NS_ZERO);
      cycleState_ = CycleState::Off;
      currentIndex_ = -1;
      return true;
    }

    return true;
  }

  switch (cycleState_) {
  case CycleState::Off:
    sendStart(BROADCAST, NS_ZERO);
    cycleState_ = CycleState::BroadcastStarted;
    currentIndex_ = -1;
    return true;

  case CycleState::BroadcastStarted:
  case CycleState::PassiveAmbient:
    sendBlackout(BROADCAST, NS_ZERO);
    currentIndex_ = 0;
    sendStart(DEFAULT_DEVICE, targets_[currentIndex_]);
    cycleState_ = CycleState::TargetStarted;
    return true;

  case CycleState::TargetStarted:
    if (currentIndex_ >= 0 &&
        currentIndex_ < static_cast<int>(targets_.size())) {
      sendBlackout(DEFAULT_DEVICE, targets_[currentIndex_]);
    }

    currentIndex_++;
    if (currentIndex_ >= static_cast<int>(targets_.size())) {
      sendBlackout(BROADCAST, NS_ZERO);
      cycleState_ = CycleState::Off;
      currentIndex_ = -1;
      return true;
    }
    sendStart(DEFAULT_DEVICE, targets_[currentIndex_]);
    cycleState_ = CycleState::TargetStarted;
    return true;
  }

  return true;
}

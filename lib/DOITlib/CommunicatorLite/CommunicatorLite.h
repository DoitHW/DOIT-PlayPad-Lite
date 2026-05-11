#pragma once

#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>

class CommunicatorLite {
public:
  void setTargets(const std::vector<TARGETNS> &targets);
  void reloadTargets();
  bool next();
  bool hasTargets() const { return !targets_.empty(); }
  uint8_t activeTargetType() const;
  TARGETNS activeTargetNS() const;

private:
  enum class CycleState : uint8_t {
    Off,
    BroadcastStarted,
    TargetStarted,
    PassiveAmbient,
  };

  void sendBlackout(uint8_t targetType, const TARGETNS &targetNS);
  void sendStart(uint8_t targetType, const TARGETNS &targetNS,
                 bool includeRelayFlag);
  void sendBroadcastAmbient();

  std::vector<TARGETNS> targets_;
  int currentIndex_ = -1;
  CycleState cycleState_ = CycleState::Off;
};

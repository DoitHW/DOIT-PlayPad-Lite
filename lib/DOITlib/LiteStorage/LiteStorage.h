#pragma once

#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>

namespace LiteStorage {

constexpr size_t kNameLen = 32;
constexpr size_t kDescLen = 192;
constexpr size_t kSerialLen = 5;
constexpr size_t kIdLen = 1;
constexpr size_t kCurrentModeLen = 1;

constexpr size_t kOffsetName = 0;
constexpr size_t kOffsetDesc = kOffsetName + kNameLen;
constexpr size_t kOffsetSerial = kOffsetDesc + kDescLen;
constexpr size_t kOffsetId = kOffsetSerial + kSerialLen;
constexpr size_t kOffsetCurrentMode = kOffsetId + kIdLen;
constexpr size_t kMinimumElementFileSize = kOffsetCurrentMode + kCurrentModeLen;

bool begin();
std::vector<TARGETNS> loadTargets();
size_t clearTargets();
bool targetExists(const TARGETNS &ns);
bool saveMinimalTarget(const TARGETNS &ns);
String targetToHex(const TARGETNS &ns);

} // namespace LiteStorage

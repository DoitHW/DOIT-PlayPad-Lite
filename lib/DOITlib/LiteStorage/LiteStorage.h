#pragma once

#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>

namespace LiteStorage {

bool begin();
std::vector<TARGETNS> loadTargets();
size_t clearTargets();
bool saveMinimalTarget(const TARGETNS &ns);

} // namespace LiteStorage

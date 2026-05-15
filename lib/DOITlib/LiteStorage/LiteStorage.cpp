#include <LiteStorage/LiteStorage.h>

#include <FS.h>
#include <SPIFFS.h>
#include <algorithm>

namespace LiteStorage {
namespace {

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

bool isElementFile(const String &path) {
  return path.startsWith("/element_") && path.endsWith(".bin") &&
         path.indexOf("_icon") < 0;
}

bool isStoredElementFile(const String &path) {
  return path.startsWith("/element_") && path.endsWith(".bin");
}

bool nsEquals(const TARGETNS &a, const TARGETNS &b) {
  return a.mac01 == b.mac01 && a.mac02 == b.mac02 && a.mac03 == b.mac03 &&
         a.mac04 == b.mac04 && a.mac05 == b.mac05;
}

bool nsIsZero(const TARGETNS &ns) {
  return (ns.mac01 | ns.mac02 | ns.mac03 | ns.mac04 | ns.mac05) == 0;
}

bool readNSFromFile(const String &path, TARGETNS &out) {
  out = NS_ZERO;
  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    return false;
  }

  if (file.size() < kMinimumElementFileSize ||
      !file.seek(kOffsetSerial, SeekSet)) {
    file.close();
    return false;
  }

  uint8_t raw[kSerialLen] = {0, 0, 0, 0, 0};
  const int readBytes = file.read(raw, sizeof(raw));
  file.close();

  if (readBytes != static_cast<int>(sizeof(raw))) {
    return false;
  }

  out = TARGETNS{raw[0], raw[1], raw[2], raw[3], raw[4]};
  return !nsIsZero(out);
}

bool writeZeroes(File &file, size_t count) {
  uint8_t zeroes[32] = {0};
  while (count > 0) {
    const size_t chunk = min(count, sizeof(zeroes));
    if (file.write(zeroes, chunk) != chunk) {
      return false;
    }
    count -= chunk;
  }
  return true;
}

String targetToHex(const TARGETNS &ns) {
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "%02X%02X%02X%02X%02X", ns.mac01, ns.mac02,
           ns.mac03, ns.mac04, ns.mac05);
  return String(buffer);
}

} // namespace

bool begin() {
  return SPIFFS.begin(true);
}

std::vector<TARGETNS> loadTargets() {
  std::vector<TARGETNS> targets;

  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) {
    return targets;
  }

  for (File file = root.openNextFile(); file; file = root.openNextFile()) {
    String path = file.name();
    file.close();
    if (!path.startsWith("/")) {
      path = "/" + path;
    }
    if (!isElementFile(path)) {
      continue;
    }

    TARGETNS ns = NS_ZERO;
    if (readNSFromFile(path, ns)) {
      targets.push_back(ns);
    }
  }
  root.close();

  std::sort(targets.begin(), targets.end(), [](const TARGETNS &a, const TARGETNS &b) {
    if (a.mac01 != b.mac01) return a.mac01 < b.mac01;
    if (a.mac02 != b.mac02) return a.mac02 < b.mac02;
    if (a.mac03 != b.mac03) return a.mac03 < b.mac03;
    if (a.mac04 != b.mac04) return a.mac04 < b.mac04;
    return a.mac05 < b.mac05;
  });
  targets.erase(std::unique(targets.begin(), targets.end(), nsEquals),
                targets.end());
  return targets;
}

size_t clearTargets() {
  std::vector<String> paths;

  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) {
    return 0;
  }

  for (File file = root.openNextFile(); file; file = root.openNextFile()) {
    String path = file.name();
    file.close();
    if (!path.startsWith("/")) {
      path = "/" + path;
    }
    if (isStoredElementFile(path)) {
      paths.push_back(path);
    }
  }
  root.close();

  size_t removed = 0;
  for (const String &path : paths) {
    if (SPIFFS.remove(path)) {
      removed++;
    }
  }
  return removed;
}

bool saveMinimalTarget(const TARGETNS &ns) {
  if (nsIsZero(ns)) {
    return false;
  }

  const String hex = targetToHex(ns);
  const String path = "/element_" + hex + ".bin";
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    return false;
  }

  uint8_t name[kNameLen] = {0};
  const String label = "NS_" + hex;
  memcpy(name, label.c_str(), min(label.length(), sizeof(name) - 1));

  bool ok = true;
  ok &= file.write(name, sizeof(name)) == sizeof(name);
  ok &= writeZeroes(file, kDescLen);

  const uint8_t serial[kSerialLen] = {ns.mac01, ns.mac02, ns.mac03, ns.mac04,
                                      ns.mac05};
  ok &= file.write(serial, sizeof(serial)) == sizeof(serial);

  const uint8_t id = 0x00;
  const uint8_t currentMode = DEFAULT_BASIC_MODE;
  ok &= file.write(&id, sizeof(id)) == sizeof(id);
  ok &= file.write(&currentMode, sizeof(currentMode)) == sizeof(currentMode);

  file.close();

  if (!ok) {
    SPIFFS.remove(path);
    return false;
  }
  return true;
}

} // namespace LiteStorage

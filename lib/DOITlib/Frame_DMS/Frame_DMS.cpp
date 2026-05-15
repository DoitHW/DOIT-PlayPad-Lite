#include <Frame_DMS/Frame_DMS.h>

#include <Arduino.h>

volatile bool frameReceived = false;
std::vector<byte> uartBuffer;

namespace {

constexpr uint16_t FRAME_HEADER_BASE_LEN = 18;
constexpr uint16_t FRAME_MIN_TOTAL_BYTES = 21;

TARGETNS localNS = NS_ZERO;
uint8_t localRoom = DEFAULT_ROOM;

void foldChecksum(uint16_t &sum) {
  while (sum > 0xFF) {
    sum = (sum & 0xFF) + (sum >> 8);
  }
}

const char *liteFunctionName(byte function) {
  switch (function) {
  case F_SEND_COLOR:
    return "F_SEND_COLOR";
  case F_SEND_PATTERN_NUM:
    return "F_SEND_PATTERN_NUM";
  case F_SEND_FLAG_BYTE:
    return "F_SEND_FLAG_BYTE";
  case F_SEND_COMMAND:
    return "F_SEND_COMMAND";
  default:
    return "UNKNOWN";
  }
}

void printFrameByte(byte value) {
  Serial.printf("%02X ", value);
}

void printFrameTx(const FRAME_T &frame) {
  const uint16_t dataLength =
      (static_cast<uint16_t>(frame.dataLengthMsb) << 8) | frame.dataLengthLsb;

  Serial.printf("[RF TX] fn=%s(0x%02X) room=0x%02X targetType=0x%02X "
                "targetNS=%02X%02X%02X%02X%02X dataLen=%u data=",
                liteFunctionName(frame.function), frame.function, frame.room,
                frame.targetType, frame.targetNS.mac01, frame.targetNS.mac02,
                frame.targetNS.mac03, frame.targetNS.mac04,
                frame.targetNS.mac05, dataLength);
  for (byte dataByte : frame.data) {
    printFrameByte(dataByte);
  }

  Serial.print("hex=");
  printFrameByte(frame.start);
  printFrameByte(frame.frameLengthMsb);
  printFrameByte(frame.frameLengthLsb);
  printFrameByte(frame.room);
  printFrameByte(frame.origin);
  printFrameByte(frame.originNS.mac01);
  printFrameByte(frame.originNS.mac02);
  printFrameByte(frame.originNS.mac03);
  printFrameByte(frame.originNS.mac04);
  printFrameByte(frame.originNS.mac05);
  printFrameByte(frame.targetType);
  printFrameByte(frame.targetNS.mac01);
  printFrameByte(frame.targetNS.mac02);
  printFrameByte(frame.targetNS.mac03);
  printFrameByte(frame.targetNS.mac04);
  printFrameByte(frame.targetNS.mac05);
  printFrameByte(frame.function);
  printFrameByte(frame.dataLengthMsb);
  printFrameByte(frame.dataLengthLsb);
  for (byte dataByte : frame.data) {
    printFrameByte(dataByte);
  }
  printFrameByte(frame.checksum);
  printFrameByte(frame.end);
  Serial.println();
}

byte checksumCalc(const FRAME_T &frame) {
  uint16_t sum = 0;
  sum += frame.start;
  sum += frame.frameLengthMsb;
  sum += frame.frameLengthLsb;
  sum += frame.room;
  sum += frame.origin;
  sum += frame.originNS.mac01;
  sum += frame.originNS.mac02;
  sum += frame.originNS.mac03;
  sum += frame.originNS.mac04;
  sum += frame.originNS.mac05;
  sum += frame.targetType;
  sum += frame.targetNS.mac01;
  sum += frame.targetNS.mac02;
  sum += frame.targetNS.mac03;
  sum += frame.targetNS.mac04;
  sum += frame.targetNS.mac05;
  sum += frame.function;
  sum += frame.dataLengthMsb;
  sum += frame.dataLengthLsb;
  for (byte dataByte : frame.data) {
    sum += dataByte;
  }
  sum += NEW_END;
  foldChecksum(sum);
  return static_cast<byte>(sum);
}

FRAME_T makeFrame(byte origin, byte targetType, TARGETNS targetNS, byte function,
                  std::initializer_list<byte> data) {
  FRAME_T frame;
  const uint16_t dataLength = static_cast<uint16_t>(data.size());
  const uint16_t frameLength = FRAME_HEADER_BASE_LEN + dataLength;

  frame.frameLengthMsb = (frameLength >> 8) & 0xFF;
  frame.frameLengthLsb = frameLength & 0xFF;
  frame.room = localRoom;
  frame.origin = origin;
  frame.originNS = localNS;
  frame.targetType = targetType;
  frame.targetNS = targetNS;
  frame.function = function;
  frame.dataLengthMsb = (dataLength >> 8) & 0xFF;
  frame.dataLengthLsb = dataLength & 0xFF;
  frame.data.assign(data.begin(), data.end());
  frame.checksum = checksumCalc(frame);
  return frame;
}

} // namespace

void setLocalNS(const TARGETNS &ns) {
  localNS = ns;
}

void setLocalRoom(uint8_t room) {
  localRoom = room;
}

void IRAM_ATTR onUartInterrupt() {
  enum ReceiveState : uint8_t {
    WAITING_START,
    RECEIVING_LENGTH_MSB,
    RECEIVING_LENGTH_LSB,
    RECEIVING_FRAME
  };

  static ReceiveState receiveState = WAITING_START;
  static uint16_t expectedFrameLength = 0;
  static uint16_t totalFrameLength = 0;
  static uint16_t receivedBytes = 0;
  static uint16_t calculatedChecksum = 0;
  static uint8_t receivedChecksum = 0;

  while (Serial1.available()) {
    const byte receivedByte = Serial1.read();

    if (receiveState == WAITING_START && receivedByte != NEW_START) {
      continue;
    }

    if (uartBuffer.size() > MAX_BUFFER_SIZE || expectedFrameLength > 0xFF) {
      receiveState = WAITING_START;
      expectedFrameLength = 0;
      uartBuffer.clear();
      return;
    }

    switch (receiveState) {
    case WAITING_START:
      uartBuffer.clear();
      uartBuffer.push_back(receivedByte);
      calculatedChecksum = receivedByte;
      expectedFrameLength = 0;
      receivedBytes = 1;
      receiveState = RECEIVING_LENGTH_MSB;
      break;

    case RECEIVING_LENGTH_MSB:
      uartBuffer.push_back(receivedByte);
      calculatedChecksum += receivedByte;
      foldChecksum(calculatedChecksum);
      expectedFrameLength = static_cast<uint16_t>(receivedByte) << 8;
      receiveState = RECEIVING_LENGTH_LSB;
      break;

    case RECEIVING_LENGTH_LSB:
      uartBuffer.push_back(receivedByte);
      calculatedChecksum += receivedByte;
      foldChecksum(calculatedChecksum);
      expectedFrameLength |= receivedByte;
      totalFrameLength = expectedFrameLength + 3;

      if (totalFrameLength < MIN_FRAME_LENGTH ||
          totalFrameLength > MAX_FRAME_LENGTH) {
        receiveState = WAITING_START;
        expectedFrameLength = 0;
        uartBuffer.clear();
        return;
      }

      receivedBytes = 3;
      receiveState = RECEIVING_FRAME;
      break;

    case RECEIVING_FRAME:
      if (uartBuffer.size() >= MAX_BUFFER_SIZE) {
        receiveState = WAITING_START;
        uartBuffer.clear();
        return;
      }

      uartBuffer.push_back(receivedByte);
      receivedBytes++;

      if (receivedBytes != totalFrameLength - 1) {
        calculatedChecksum += receivedByte;
      } else {
        receivedChecksum = receivedByte;
      }
      foldChecksum(calculatedChecksum);

      if (receivedBytes == totalFrameLength) {
        if (receivedByte == NEW_END && calculatedChecksum == receivedChecksum &&
            uartBuffer.size() >= FRAME_MIN_TOTAL_BYTES) {
          const uint8_t frameRoom = uartBuffer[3];
          const bool roomMatches =
              frameRoom == ROOM_BROADCAST || frameRoom == localRoom;
          frameReceived = roomMatches;
        } else {
          frameReceived = false;
        }

        receiveState = WAITING_START;
        return;
      }
      break;
    }
  }
}

LAST_ENTRY_FRAME_T extract_info_from_frameIn(const std::vector<uint8_t> &frame) {
  LAST_ENTRY_FRAME_T result;
  if (frame.size() < FRAME_MIN_TOTAL_BYTES) {
    return result;
  }

  constexpr size_t IDX_ROOM = 3;
  constexpr size_t IDX_ORIGIN = 4;
  constexpr size_t IDX_ORIGIN_NS = 5;
  constexpr size_t IDX_TARGET_TYPE = 10;
  constexpr size_t IDX_TARGET_NS = 11;
  constexpr size_t IDX_FUNCTION = 16;
  constexpr size_t IDX_DLEN_MSB = 17;
  constexpr size_t IDX_DLEN_LSB = 18;
  constexpr size_t IDX_DATA_START = 19;

  result.room = frame[IDX_ROOM];
  result.origin = frame[IDX_ORIGIN];
  result.originNS = {frame[IDX_ORIGIN_NS + 0], frame[IDX_ORIGIN_NS + 1],
                     frame[IDX_ORIGIN_NS + 2], frame[IDX_ORIGIN_NS + 3],
                     frame[IDX_ORIGIN_NS + 4]};
  result.targetType = frame[IDX_TARGET_TYPE];
  result.targetNS = {frame[IDX_TARGET_NS + 0], frame[IDX_TARGET_NS + 1],
                     frame[IDX_TARGET_NS + 2], frame[IDX_TARGET_NS + 3],
                     frame[IDX_TARGET_NS + 4]};
  result.function = frame[IDX_FUNCTION];

  const uint16_t dataLength =
      (static_cast<uint16_t>(frame[IDX_DLEN_MSB]) << 8) | frame[IDX_DLEN_LSB];
  if (IDX_DATA_START + dataLength <= frame.size()) {
    result.data.assign(frame.begin() + IDX_DATA_START,
                       frame.begin() + IDX_DATA_START + dataLength);
  }

  return result;
}

void send_frame(const FRAME_T &frame) {
  printFrameTx(frame);

  Serial1.write(frame.start);
  Serial1.write(frame.frameLengthMsb);
  Serial1.write(frame.frameLengthLsb);
  Serial1.write(frame.room);
  Serial1.write(frame.origin);
  Serial1.write(frame.originNS.mac01);
  Serial1.write(frame.originNS.mac02);
  Serial1.write(frame.originNS.mac03);
  Serial1.write(frame.originNS.mac04);
  Serial1.write(frame.originNS.mac05);
  Serial1.write(frame.targetType);
  Serial1.write(frame.targetNS.mac01);
  Serial1.write(frame.targetNS.mac02);
  Serial1.write(frame.targetNS.mac03);
  Serial1.write(frame.targetNS.mac04);
  Serial1.write(frame.targetNS.mac05);
  Serial1.write(frame.function);
  Serial1.write(frame.dataLengthMsb);
  Serial1.write(frame.dataLengthLsb);
  for (byte dataByte : frame.data) {
    Serial1.write(dataByte);
  }
  Serial1.write(frame.checksum);
  Serial1.write(frame.end);
  Serial1.flush();

  Serial.printf("[RF TX DONE] UART flushed fn=%s(0x%02X) bytes=%u baud=%lu\n",
                liteFunctionName(frame.function), frame.function,
                static_cast<unsigned>(21 + frame.data.size()),
                static_cast<unsigned long>(RF_BAUD_RATE));

}

FRAME_T frameMaker_SEND_COLOR(byte originin, byte targetType, TARGETNS targetNS,
                              byte colorin) {
  return makeFrame(originin, targetType, targetNS, F_SEND_COLOR, {colorin});
}

FRAME_T frameMaker_SEND_COMMAND(byte originin, byte targetType,
                                TARGETNS targetNS, byte commandin) {
  return makeFrame(originin, targetType, targetNS, F_SEND_COMMAND, {commandin});
}

FRAME_T frameMaker_SEND_FLAG_BYTE(byte originin, byte targetType,
                                  TARGETNS targetNS, byte flagin) {
  return makeFrame(originin, targetType, targetNS, F_SEND_FLAG_BYTE, {flagin});
}

FRAME_T frameMaker_SEND_PATTERN_NUM(byte originin, byte targetType,
                                    TARGETNS targetNS, byte patternin) {
  return makeFrame(originin, targetType, targetNS, F_SEND_PATTERN_NUM,
                   {patternin});
}

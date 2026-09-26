#include "LD2415H.h"

#include <cstdio>

namespace hlk {
namespace ld2415h {

static const char *const kTag = "ld2415h";

namespace {

// Command frames (see HLK-LD2415H datasheet, "Serial Interface Command Syntax").
// Bytes are mutated before sending, so each frame is copied from a constant template.
const uint8_t kCmdSetSpeedAngleSense[8] = {0x43, 0x46, 0x01, 0x01, 0x00, 0x05, 0x0d, 0x0a};
const uint8_t kCmdSetModeRateUom[8] = {0x43, 0x46, 0x02, 0x01, 0x01, 0x00, 0x0d, 0x0a};
const uint8_t kCmdSetAntiVibComp[8] = {0x43, 0x46, 0x03, 0x05, 0x00, 0x00, 0x0d, 0x0a};
const uint8_t kCmdSetRelayDurationSpeed[8] = {0x43, 0x46, 0x04, 0x03, 0x01, 0x00, 0x0d, 0x0a};
const uint8_t kCmdGetConfig[13] = {0x43, 0x46, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const std::map<std::string, uint8_t> kNegotiationModeStrings{
    {"Custom Agreement", static_cast<uint8_t>(NegotiationMode::CUSTOM_AGREEMENT)},
    {"Standard Protocol", static_cast<uint8_t>(NegotiationMode::STANDARD_PROTOCOL)}};

const std::map<std::string, uint8_t> kSampleRateStrings{
    {"~22 fps", static_cast<uint8_t>(SampleRate::SAMPLE_RATE_22FPS)},
    {"~11 fps", static_cast<uint8_t>(SampleRate::SAMPLE_RATE_11FPS)},
    {"~6 fps", static_cast<uint8_t>(SampleRate::SAMPLE_RATE_6FPS)}};

const std::map<std::string, uint8_t> kTrackingModeStrings{
    {"Approaching and Retreating", static_cast<uint8_t>(TrackingMode::APPROACHING_AND_RETREATING)},
    {"Approaching", static_cast<uint8_t>(TrackingMode::APPROACHING)},
    {"Retreating", static_cast<uint8_t>(TrackingMode::RETREATING)}};

const std::map<std::string, uint8_t> kUnitOfMeasureStrings{
    {"km/h", static_cast<uint8_t>(UnitOfMeasure::KPH)}, {"mph", static_cast<uint8_t>(UnitOfMeasure::MPH)}, {"m/s", static_cast<uint8_t>(UnitOfMeasure::MPS)}};

}  // namespace

const std::map<std::string, uint8_t> &negotiationModeStrings() { return kNegotiationModeStrings; }
const std::map<std::string, uint8_t> &sampleRateStrings() { return kSampleRateStrings; }
const std::map<std::string, uint8_t> &trackingModeStrings() { return kTrackingModeStrings; }
const std::map<std::string, uint8_t> &unitOfMeasureStrings() { return kUnitOfMeasureStrings; }

const char *enumToString(const std::map<std::string, uint8_t> &values, uint8_t value) {
  for (const auto &pair : values) {
    if (pair.second == value) {
      return pair.first.c_str();
    }
  }
  return "Unknown";
}

LD2415H::LD2415H(Transport *transport, Logger *logger)
    : transport_(transport), logger_(logger != nullptr ? logger : new NullLogger()) {
  std::memset(responseBuffer_, 0, sizeof(responseBuffer_));
}

void LD2415H::update() {
  // Process the stream from the sensor UART.
  while (transport_->available() > 0) {
    if (fillBuffer(static_cast<char>(transport_->read()))) {
      parseBuffer();
    }
  }

  if (updateSpeedAngleSense_) {
    uint8_t cmd[sizeof(kCmdSetSpeedAngleSense)];
    std::memcpy(cmd, kCmdSetSpeedAngleSense, sizeof(cmd));
    cmd[3] = config_.minSpeedThreshold;
    cmd[4] = config_.compensationAngle;
    cmd[5] = config_.sensitivity;
    issueCommand(cmd, sizeof(cmd));
    updateSpeedAngleSense_ = false;
    return;
  }

  if (updateModeRateUom_) {
    uint8_t cmd[sizeof(kCmdSetModeRateUom)];
    std::memcpy(cmd, kCmdSetModeRateUom, sizeof(cmd));
    cmd[3] = static_cast<uint8_t>(config_.trackingMode);
    cmd[4] = config_.sampleRate;
    issueCommand(cmd, sizeof(cmd));
    updateModeRateUom_ = false;
    return;
  }

  if (updateAntiVibComp_) {
    uint8_t cmd[sizeof(kCmdSetAntiVibComp)];
    std::memcpy(cmd, kCmdSetAntiVibComp, sizeof(cmd));
    cmd[3] = config_.vibrationCorrection;
    issueCommand(cmd, sizeof(cmd));
    updateAntiVibComp_ = false;
    return;
  }

  if (updateRelayDurationSpeed_) {
    uint8_t cmd[sizeof(kCmdSetRelayDurationSpeed)];
    std::memcpy(cmd, kCmdSetRelayDurationSpeed, sizeof(cmd));
    cmd[3] = config_.relayTriggerDuration;
    cmd[4] = config_.relayTriggerSpeed;
    issueCommand(cmd, sizeof(cmd));
    updateRelayDurationSpeed_ = false;
    return;
  }

  if (updateConfig_) {
    issueCommand(kCmdGetConfig, sizeof(kCmdGetConfig));
    updateConfig_ = false;
    return;
  }
}

void LD2415H::unregisterListener(Listener *listener) {
  for (auto it = listeners_.begin(); it != listeners_.end(); ++it) {
    if (*it == listener) {
      listeners_.erase(it);
      return;
    }
  }
}

void LD2415H::setMinSpeedThreshold(uint8_t value) {
  config_.minSpeedThreshold = value;
  updateSpeedAngleSense_ = true;
}

void LD2415H::setCompensationAngle(uint8_t value) {
  config_.compensationAngle = value;
  updateSpeedAngleSense_ = true;
}

void LD2415H::setSensitivity(uint8_t value) {
  config_.sensitivity = value;
  updateSpeedAngleSense_ = true;
}

void LD2415H::setTrackingMode(TrackingMode mode) {
  config_.trackingMode = mode;
  updateModeRateUom_ = true;
}

void LD2415H::setSampleRate(uint8_t rate) {
  config_.sampleRate = rate;
  updateModeRateUom_ = true;
}

void LD2415H::setVibrationCorrection(uint8_t value) {
  config_.vibrationCorrection = value;
  updateAntiVibComp_ = true;
}

void LD2415H::setRelayTriggerDuration(uint8_t value) {
  config_.relayTriggerDuration = value;
  updateRelayDurationSpeed_ = true;
}

void LD2415H::setRelayTriggerSpeed(uint8_t value) {
  config_.relayTriggerSpeed = value;
  updateRelayDurationSpeed_ = true;
}

void LD2415H::requestConfig() { updateConfig_ = true; }

void LD2415H::issueCommand(const uint8_t *cmd, uint8_t size) {
  logDebug("tx:");
  for (uint8_t i = 0; i < size; i++) {
    char line[16];
    std::snprintf(line, sizeof(line), "  0x%02x", cmd[i]);
    logDebug(line);
  }
  // Don't assume the response buffer is empty, clear it before issuing a command.
  clearRemainingBuffer(0);
  transport_->write(cmd, size);
}

bool LD2415H::fillBuffer(char c) {
  switch (static_cast<unsigned char>(c)) {
    case 0x00:
    case 0xFF:
    case '\r':
      // Ignore these characters
      break;

    case '\n':
      // End of response
      if (responseBufferIndex_ == 0)
        break;

      clearRemainingBuffer(responseBufferIndex_);
      log(LogLevel::DEBUG, responseBuffer_);
      return true;

    default:
      // Append to response. Guard against overflow of the fixed buffer.
      if (responseBufferIndex_ < sizeof(responseBuffer_) - 1) {
        responseBuffer_[responseBufferIndex_] = c;
        responseBufferIndex_++;
      }
      break;
  }

  return false;
}

void LD2415H::clearRemainingBuffer(uint8_t pos) {
  while (pos < sizeof(responseBuffer_)) {
    responseBuffer_[pos] = 0x00;
    pos++;
  }

  responseBufferIndex_ = 0;
}

void LD2415H::parseBuffer() {
  char c = responseBuffer_[0];

  switch (c) {
    case 'N':
      // Firmware Version
      parseFirmware();
      break;
    case 'X':
      // Config Response
      parseConfig();
      break;
    case 'V':
      // Speed
      parseSpeed();
      break;

    default:
      logError(responseBuffer_);
      break;
  }
}

void LD2415H::parseConfig() {
  // Example: "X1:01 X2:00 X3:05 X4:01 X5:00 X6:00 X7:05 X8:03 X9:01 X0:01"

  const char *delim = ": ";
  const uint8_t tokenLen = 2;
  char *key;
  char *val;

  char *token = std::strtok(responseBuffer_, delim);

  while (token != nullptr) {
    if (std::strlen(token) != tokenLen) {
      logError("Configuration key length invalid.");
      break;
    }
    key = token;

    token = std::strtok(nullptr, delim);
    if (token == nullptr) {
      logError("Configuration value missing.");
      break;
    }
    if (std::strlen(token) != tokenLen) {
      logError("Configuration value length invalid.");
      break;
    }
    val = token;

    parseConfigParam(key, val);

    token = std::strtok(nullptr, delim);
  }

  for (auto &listener : listeners_) {
    listener->onConfig();
  }
}

void LD2415H::parseFirmware() {
  // Example: "No.:20230801E v5.0"

  const char *fw = std::strchr(responseBuffer_, ':');

  if (fw != nullptr) {
    // Move past the ':'
    ++fw;

    // Strip trailing whitespace/control chars the sensor appends.
    firmware_.assign(fw);
    while (!firmware_.empty() && (firmware_.back() == '\r' || firmware_.back() == '\n' ||
                                  firmware_.back() == ' ' ||
                                  static_cast<unsigned char>(firmware_.back()) == 0xff)) {
      firmware_.pop_back();
    }
  } else {
    logError("Firmware value invalid.");
  }
}

void LD2415H::parseSpeed() {
  // Example: "V+001.9"

  const char *p = std::strchr(responseBuffer_, 'V');

  if (p != nullptr) {
    ++p;
    velocity_ = static_cast<float>(std::strtod(p, nullptr));
    p += 1;  // skip the sign of the speed value
    speed_ = static_cast<float>(std::strtod(p, nullptr));

    log(LogLevel::DEBUG, "Speed updated");
    notifyListeners();
  } else {
    logError("Speed value invalid.");
  }
}

void LD2415H::notifyListeners() {
  for (auto &listener : listeners_) {
    listener->onSpeed(speed_);
    listener->onVelocity(velocity_);
  }
}

void LD2415H::parseConfigParam(char *key, char *value) {
  if (std::strlen(key) != 2 || std::strlen(value) != 2 || key[0] != 'X') {
    char line[32];
    std::snprintf(line, sizeof(line), "Invalid Parameter %s:%s", key, value);
    logError(line);
    return;
  }

  uint8_t v = static_cast<uint8_t>(std::stoi(value, nullptr, 16));

  switch (key[1]) {
    case '1':
      config_.minSpeedThreshold = v;
      break;
    case '2':
      config_.compensationAngle = static_cast<uint8_t>(std::stoi(value, nullptr, 16));
      break;
    case '3':
      config_.sensitivity = static_cast<uint8_t>(std::stoi(value, nullptr, 16));
      break;
    case '4':
      config_.trackingMode = iToTrackingMode(v);
      break;
    case '5':
      config_.sampleRate = v;
      break;
    case '6':
      config_.unitOfMeasure = iToUnitOfMeasure(v);
      break;
    case '7':
      config_.vibrationCorrection = v;
      break;
    case '8':
      config_.relayTriggerDuration = v;
      break;
    case '9':
      config_.relayTriggerSpeed = v;
      break;
    case '0':
      config_.negotiationMode = iToNegotiationMode(v);
      break;
    default:
      break;
  }
}

TrackingMode LD2415H::iToTrackingMode(uint8_t value) {
  switch (static_cast<TrackingMode>(value)) {
    case TrackingMode::APPROACHING_AND_RETREATING:
      return TrackingMode::APPROACHING_AND_RETREATING;
    case TrackingMode::APPROACHING:
      return TrackingMode::APPROACHING;
    case TrackingMode::RETREATING:
      return TrackingMode::RETREATING;
    default:
      logError("Invalid TrackingMode");
      return TrackingMode::APPROACHING_AND_RETREATING;
  }
}

UnitOfMeasure LD2415H::iToUnitOfMeasure(uint8_t value) {
  switch (static_cast<UnitOfMeasure>(value)) {
    case UnitOfMeasure::MPS:
      return UnitOfMeasure::MPS;
    case UnitOfMeasure::MPH:
      return UnitOfMeasure::MPH;
    case UnitOfMeasure::KPH:
      return UnitOfMeasure::KPH;
    default:
      logError("Invalid UnitOfMeasure");
      return UnitOfMeasure::KPH;
  }
}

NegotiationMode LD2415H::iToNegotiationMode(uint8_t value) {
  switch (static_cast<NegotiationMode>(value)) {
    case NegotiationMode::CUSTOM_AGREEMENT:
      return NegotiationMode::CUSTOM_AGREEMENT;
    case NegotiationMode::STANDARD_PROTOCOL:
      return NegotiationMode::STANDARD_PROTOCOL;
    default:
      logError("Invalid NegotiationMode");
      return NegotiationMode::CUSTOM_AGREEMENT;
  }
}

void LD2415H::log(LogLevel level, const char *message) {
  if (logger_ != nullptr) {
    logger_->log(level, kTag, message);
  }
}

void LD2415H::logDebug(const char *message) { log(LogLevel::DEBUG, message); }
void LD2415H::logError(const char *message) { log(LogLevel::ERROR, message); }

}  // namespace ld2415h
}  // namespace hlk

#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace ld2415h {

// Log levels
enum class LogLevel : uint8_t {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
};

// Minimal UART-like transport the protocol engine reads/writes through.
// For tests, anything that can feed bytes in and take bytes out will do.
class Transport {
 public:
  virtual ~Transport() = default;

  // Number of bytes currently available to read.
  virtual int available() = 0;

  // Read one byte. Only call when available() > 0.
  virtual int read() = 0;

  // Write a complete command.
  virtual void write(const uint8_t *data, uint8_t size) = 0;
};

// Host-provided log sink. The library never prints on its own.
class Logger {
 public:
  virtual ~Logger() = default;
  virtual void log(LogLevel level, const char *tag, const char *message) = 0;
};

// Default: discard everything.
class NullLogger : public Logger {
 public:
  void log(LogLevel, const char *, const char *) override {}
};

enum class NegotiationMode : uint8_t { CUSTOM_AGREEMENT = 0x01, STANDARD_PROTOCOL = 0x02 };

enum class SampleRate : uint8_t { SAMPLE_RATE_22FPS = 0x00, SAMPLE_RATE_11FPS = 0x01, SAMPLE_RATE_6FPS = 0x02 };

enum class TrackingMode : uint8_t { APPROACHING_AND_RETREATING = 0x00, APPROACHING = 0x01, RETREATING = 0x02 };

enum class UnitOfMeasure : uint8_t { KPH = 0x00, MPH = 0x01, MPS = 0x02 };

const std::map<std::string, uint8_t> &negotiationModeStrings();
const std::map<std::string, uint8_t> &sampleRateStrings();
const std::map<std::string, uint8_t> &trackingModeStrings();
const std::map<std::string, uint8_t> &unitOfMeasureStrings();

// Human-readable name for an enum value, "Unknown" if unrecognized.
const char *enumToString(const std::map<std::string, uint8_t> &values, uint8_t value);

// Callbacks for parsed sensor data. Default implementations are no-ops.
class Listener {
 public:
  virtual ~Listener() = default;
  // Unsigned speed in km/h.
  virtual void onSpeed(float) {}
  // Signed velocity in km/h (positive = approaching, negative = retreating).
  virtual void onVelocity(float) {}
  // Called after the sensor config block (X1..X0) has been parsed.
  virtual void onConfig() {}
};

// Parsed configuration reported by the sensor (see datasheet, command 0x07).
struct Configuration {
  uint8_t minSpeedThreshold = 1;
  uint8_t compensationAngle = 0;
  uint8_t sensitivity = 10;
  TrackingMode trackingMode = TrackingMode::APPROACHING_AND_RETREATING;
  uint8_t sampleRate = 1;
  UnitOfMeasure unitOfMeasure = UnitOfMeasure::KPH;
  uint8_t vibrationCorrection = 18;
  uint8_t relayTriggerDuration = 0;
  uint8_t relayTriggerSpeed = 1;
  NegotiationMode negotiationMode = NegotiationMode::CUSTOM_AGREEMENT;

  void reset() { *this = Configuration(); }
};

// Transport-agnostic LD2415H protocol engine.
//
// Owns the command state machine and response parser. Call update()
// from your application loop; it drains the transport, parses complete
// responses, and issues any pending configuration commands one per call.
class LD2415H {
 public:
  explicit LD2415H(Transport *transport, Logger *logger = nullptr);

  // Feed the transport and issue pending commands. Call once per loop.
  void update();

  // Register / unregister data listeners.
  void registerListener(Listener *listener) { listeners_.push_back(listener); }
  void unregisterListener(Listener *listener);

  // ---- Configuration setters -----------------------------------------
  // Each setter stages a command; it is issued on a subsequent update()
  // call. Parameters that share a command frame (0x01: speed threshold,
  // compensation angle, sensitivity; 0x02: tracking mode, sample rate)
  // are sent together when the first of them changes.
  void setMinSpeedThreshold(uint8_t value);
  void setCompensationAngle(uint8_t value);
  void setSensitivity(uint8_t value);
  void setTrackingMode(TrackingMode mode);
  void setSampleRate(uint8_t rate);
  void setVibrationCorrection(uint8_t value);
  void setRelayTriggerDuration(uint8_t value);
  void setRelayTriggerSpeed(uint8_t value);

  // Request a full config read (command 0x07) and firmware version.
  // Parsed results arrive via onConfig()/getConfiguration()/getFirmwareVersion().
  void requestConfig();

  // ---- Read state ------------------------------------------------------
  const Configuration &getConfiguration() const { return config_; }
  const std::string &getFirmwareVersion() const { return firmware_; }
  float getSpeed() const { return speed_; }
  float getVelocity() const { return velocity_; }

 private:
  void issueCommand(const uint8_t *cmd, uint8_t size);
  bool fillBuffer(char c);
  void clearRemainingBuffer(uint8_t pos);
  void parseBuffer();
  void parseConfig();
  void parseConfigParam(char *key, char *value);
  void parseFirmware();
  void parseSpeed();
  void notifyListeners();

  TrackingMode iToTrackingMode(uint8_t value);
  UnitOfMeasure iToUnitOfMeasure(uint8_t value);
  NegotiationMode iToNegotiationMode(uint8_t value);

  void log(LogLevel level, const char *message);
  void logDebug(const char *message);
  void logError(const char *message);

  Transport *transport_;
  Logger *logger_;
  std::vector<Listener *> listeners_;

  Configuration config_;
  std::string firmware_;
  float speed_ = 0.0f;
  float velocity_ = 0.0f;

  // Pending command staging, mirrors the original per-frame flags.
  bool updateSpeedAngleSense_ = true;
  bool updateModeRateUom_ = true;
  bool updateAntiVibComp_ = true;
  bool updateRelayDurationSpeed_ = true;
  bool updateConfig_ = false;

  // Response line buffer, NUL-terminated.
  char responseBuffer_[64];
  uint8_t responseBufferIndex_ = 0;
};

}  // namespace ld2415h

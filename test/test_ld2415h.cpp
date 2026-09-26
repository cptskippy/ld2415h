// Unit tests for the LD2415H protocol library.
// Build: make test   (see Makefile)

#include "LD2415H.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace hlk::ld2415h;

// ---------------------------------------------------------------- harness

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                                     \
  do {                                                                                  \
    g_checks++;                                                                         \
    if (!(cond)) {                                                                      \
      g_failures++;                                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                      \
    }                                                                                   \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                                           \
  do {                                                                                  \
    g_checks++;                                                                         \
    if (std::fabs((a) - (b)) > (eps)) {                                                 \
      g_failures++;                                                                     \
      std::printf("FAIL %s:%d: %s (%f) != %s (%f)\n", __FILE__, __LINE__, #a, (a), #b,  \
                  (b));                                                                 \
    }                                                                                   \
  } while (0)

// ------------------------------------------------- fake transport + logger

// Records every command written, feeds canned response bytes to update().
class FakeTransport : public Transport {
 public:
  int available() override { return static_cast<int>(rx_.size()); }

  int read() override {
    int b = rx_.front();
    rx_.erase(rx_.begin());
    return b;
  }

  void write(const uint8_t *data, uint8_t size) override {
    std::vector<uint8_t> cmd(data, data + size);
    tx_.push_back(cmd);
  }

  // Queue response bytes the sensor would send.
  void feed(const std::string &s) { rx_.insert(rx_.end(), s.begin(), s.end()); }

  // All commands issued so far, in order.
  const std::vector<std::vector<uint8_t>> &commands() const { return tx_; }

  // Pop the most recent command.
  std::vector<uint8_t> popCommand() {
    auto cmd = tx_.back();
    tx_.pop_back();
    return cmd;
  }

  void reset() {
    tx_.clear();
    rx_.clear();
  }

 private:
  std::vector<uint8_t> rx_;
  std::vector<std::vector<uint8_t>> tx_;
};

// Captures log messages so tests can assert on error paths.
class CaptureLogger : public Logger {
 public:
  void log(LogLevel level, const char *, const char *message) override {
    std::string m = message;
    if (level == LogLevel::ERROR)
      errors.push_back(m);
    else
      messages.push_back(m);
  }

  std::vector<std::string> messages;
  std::vector<std::string> errors;
};

// Listener that records speed/velocity/config callbacks.
class TestListener : public Listener {
 public:
  void onSpeed(float speed) override {
    speeds.push_back(speed);
    speedCalls++;
  }
  void onVelocity(float velocity) override {
    velocities.push_back(velocity);
    velocityCalls++;
  }
  void onConfig() override { configCalls++; }

  std::vector<float> speeds;
  std::vector<float> velocities;
  int speedCalls = 0;
  int velocityCalls = 0;
  int configCalls = 0;
};

// ------------------------------------------------------------------ tests

// Initial update() issues the four staged config frames, in order.
static void testInitialCommandSequence() {
  FakeTransport t;
  CaptureLogger l;
  LD2415H radar(&t, &l);

  radar.update();
  CHECK(t.popCommand() == (std::vector<uint8_t>{0x43, 0x46, 0x01, 0x01, 0x00, 0x0A, 0x0d, 0x0a}));

  radar.update();
  CHECK(t.popCommand() == (std::vector<uint8_t>{0x43, 0x46, 0x02, 0x00, 0x01, 0x00, 0x0d, 0x0a}));

  radar.update();
  CHECK(t.popCommand() == (std::vector<uint8_t>{0x43, 0x46, 0x03, 0x12, 0x00, 0x00, 0x0d, 0x0a}));

  radar.update();
  CHECK(t.popCommand() == (std::vector<uint8_t>{0x43, 0x46, 0x04, 0x00, 0x01, 0x00, 0x0d, 0x0a}));

  // Nothing pending after the initial four frames.
  radar.update();
  CHECK(t.commands().empty());
  CHECK(l.errors.empty());
}

// Setters stage their frame with the new values.
static void testSetterStaging() {
  FakeTransport t;
  LD2415H radar(&t);
  radar.update();
  radar.update();
  radar.update();
  radar.update();
  t.reset();

  radar.setMinSpeedThreshold(0x21);
  radar.setCompensationAngle(0x32);
  radar.setSensitivity(0x43);
  radar.update();
  auto cmd = t.popCommand();
  CHECK(cmd.size() == 8 && cmd[2] == 0x01);
  CHECK(cmd[3] == 0x21 && cmd[4] == 0x32 && cmd[5] == 0x43);

  radar.setTrackingMode(TrackingMode::RETREATING);
  radar.setSampleRate(0x02);
  radar.update();
  cmd = t.popCommand();
  CHECK(cmd.size() == 8 && cmd[2] == 0x02);
  CHECK(cmd[3] == 0x02 && cmd[4] == 0x02);
}

// Full config read: request, parse the X1..X0 block, verify state.
static void testConfigParsing() {
  FakeTransport t;
  CaptureLogger l;
  TestListener listener;
  LD2415H radar(&t, &l);
  radar.registerListener(&listener);

  // Drain the four initial frames.
  for (int i = 0; i < 4; i++) radar.update();
  t.reset();

  radar.requestConfig();
  radar.update();
  auto cmd = t.popCommand();
  CHECK(cmd.size() == 13 && cmd[0] == 0x43 && cmd[1] == 0x46 && cmd[2] == 0x07);
  CHECK(cmd[3] == 0x00 && cmd[12] == 0x00);

  // Response exactly as in the datasheet, including the firmware line.
  t.feed("No.:20230801E v5.0\r\n");
  t.feed("X1:01 X2:00 X3:05 X4:01 X5:00 X6:02 X7:05 X8:03 X9:01 X0:01\r\n");
  radar.update();

  const auto &cfg = radar.getConfiguration();
  CHECK(cfg.minSpeedThreshold == 0x01);
  CHECK(cfg.compensationAngle == 0x00);
  CHECK(cfg.sensitivity == 0x05);
  CHECK(cfg.trackingMode == TrackingMode::APPROACHING);
  CHECK(cfg.sampleRate == 0x00);
  CHECK(cfg.unitOfMeasure == UnitOfMeasure::MPS);
  CHECK(cfg.vibrationCorrection == 0x05);
  CHECK(cfg.relayTriggerDuration == 0x03);
  CHECK(cfg.relayTriggerSpeed == 0x01);
  CHECK(cfg.negotiationMode == NegotiationMode::CUSTOM_AGREEMENT);
  CHECK(listener.configCalls == 1);
  CHECK(radar.getFirmwareVersion() == "20230801E v5.0");
  CHECK(l.errors.empty());
}

// Speed line: sign split between velocity and speed, listener notified.
static void testSpeedParsing() {
  FakeTransport t;
  TestListener listener;
  LD2415H radar(&t);
  radar.registerListener(&listener);
  for (int i = 0; i < 4; i++) radar.update();

  t.feed("V+012.5\r\n");
  radar.update();

  CHECK_NEAR(radar.getVelocity(), 12.5f, 0.01f);
  CHECK_NEAR(radar.getSpeed(), 12.5f, 0.01f);
  CHECK(listener.speedCalls == 1);
  CHECK(listener.velocityCalls == 1);
  CHECK_NEAR(listener.speeds[0], 12.5f, 0.01f);
  // Velocity keeps its sign: "+012.5" is approaching.
  CHECK(listener.velocities[0] > 0.0f);
  CHECK(listener.speeds[0] >= 0.0f);

  t.feed("V-042.0\r\n");
  radar.update();
  CHECK_NEAR(radar.getVelocity(), -42.0f, 0.01f);
  CHECK_NEAR(radar.getSpeed(), 42.0f, 0.01f);
  CHECK(listener.speedCalls == 2);
  CHECK(listener.speeds[1] > 0.0f);
}

// A response line may arrive split across many update() calls.
static void testFramingAcrossUpdates() {
  FakeTransport t;
  TestListener listener;
  LD2415H radar(&t);
  radar.registerListener(&listener);
  for (int i = 0; i < 4; i++) radar.update();

  const std::string line = "X1:0F X2:1E X3:0A X4:02 X5:01 X6:01 X7:20 X8:07 X9:05 X0:01\r\n";
  // Feed one byte per update() call.
  for (char c : line) {
    t.feed(std::string(1, c));
    radar.update();
  }

  const auto &cfg = radar.getConfiguration();
  CHECK(cfg.minSpeedThreshold == 0x0F);
  CHECK(cfg.compensationAngle == 0x1E);
  CHECK(cfg.sensitivity == 0x0A);
  CHECK(cfg.trackingMode == TrackingMode::RETREATING);
  CHECK(cfg.unitOfMeasure == UnitOfMeasure::MPH);
  CHECK(cfg.vibrationCorrection == 0x20);
  CHECK(cfg.relayTriggerDuration == 0x07);
  CHECK(cfg.relayTriggerSpeed == 0x05);
  CHECK(cfg.negotiationMode == NegotiationMode::CUSTOM_AGREEMENT);
  CHECK(listener.configCalls == 1);
}

// Junk / unknown lines are logged as errors and do not corrupt state.
static void testUnknownResponses() {
  FakeTransport t;
  CaptureLogger l;
  LD2415H radar(&t, &l);
  for (int i = 0; i < 4; i++) radar.update();

  t.feed("garbage line\r\n");
  radar.update();
  CHECK(l.errors.size() == 1);

  // Empty line (\n only) is ignored without error.
  size_t errorsBefore = l.errors.size();
  t.feed("\r\n");
  radar.update();
  CHECK(l.errors.size() == errorsBefore);

  // NUL bytes mid-line are skipped by fillBuffer.
  // (Feed via explicit length: the std::string(const char*) conversion
  // would stop at the embedded NUL.)
  l.errors.clear();
  t.feed(std::string("V\x00+003.0\r\n", 10));
  radar.update();
  CHECK(l.errors.empty());
  CHECK_NEAR(radar.getSpeed(), 3.0f, 0.01f);
}

// A malformed config value (wrong token length) is rejected loudly.
static void testMalformedConfig() {
  FakeTransport t;
  CaptureLogger l;
  LD2415H radar(&t, &l);
  for (int i = 0; i < 4; i++) radar.update();

  t.feed("X1:01 X2:XYZ X3:05\r\n");
  radar.update();
  CHECK(l.errors.size() >= 1);
}

// Oversized response lines cannot overflow the fixed buffer.
static void testBufferOverflow() {
  FakeTransport t;
  CaptureLogger l;
  LD2415H radar(&t, &l);
  for (int i = 0; i < 4; i++) radar.update();

  std::string huge(200, 'A');
  huge.push_back('\r');
  huge.push_back('\n');
  t.feed(huge);
  radar.update();
  // Must not crash. The overlong line is truncated to fit the buffer and
  // reported as an unknown response; no state is corrupted.
  CHECK(radar.getSpeed() == 0.0f);
  CHECK(!l.errors.empty());
}

// Listener registration and unregistration.
static void testListenerManagement() {
  FakeTransport t;
  TestListener a, b;
  LD2415H radar(&t);
  radar.registerListener(&a);
  radar.registerListener(&b);
  radar.unregisterListener(&a);
  for (int i = 0; i < 4; i++) radar.update();

  t.feed("V+005.0\r\n");
  radar.update();
  CHECK(a.speedCalls == 0);
  CHECK(b.speedCalls == 1);

  radar.unregisterListener(&b);
  t.feed("V+005.0\r\n");
  radar.update();
  CHECK(b.speedCalls == 1);
}

// enumToString round-trips known values and reports unknown ones.
static void testEnumToString() {
  CHECK(std::string(enumToString(trackingModeStrings(), 0x00)) == "Approaching and Retreating");
  CHECK(std::string(enumToString(trackingModeStrings(), 0x02)) == "Retreating");
  CHECK(std::string(enumToString(trackingModeStrings(), 0x7F)) == "Unknown");
  CHECK(std::string(enumToString(unitOfMeasureStrings(), 0x01)) == "mph");
  CHECK(std::string(enumToString(sampleRateStrings(), 0x02)) == "~6 fps");
  CHECK(std::string(enumToString(negotiationModeStrings(), 0x02)) == "Standard Protocol");
}

int main() {
  testInitialCommandSequence();
  testSetterStaging();
  testConfigParsing();
  testSpeedParsing();
  testFramingAcrossUpdates();
  testUnknownResponses();
  testMalformedConfig();
  testBufferOverflow();
  testListenerManagement();
  testEnumToString();

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

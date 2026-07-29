/*
 * fixture_hal.h — Thin hardware abstraction for future actuators/sensors.
 * Stepper and load cell remain the concrete implementations today; new devices
 * can follow the same interfaces without rewriting the command layer.
 */
#pragma once

#include <Arduino.h>

/** Bump when LINK_UART line format changes; display ESP32 can gate on this. */
constexpr uint8_t LINK_PROTO_VERSION = 1;

/** Forward declarations — defined in the main sketch */
class AccelStepper;
class HX711_MP;

/** Minimal actuator: enable and periodic poll (stepper uses run()/runSpeed() in its task). */
struct IActuator {
  virtual ~IActuator() = default;
  virtual void setEnabled(bool on) = 0;
};

/** Normalized or engineering-unit reading. */
struct ISensor {
  virtual ~ISensor() = default;
  virtual float readPrimary(int samples = 1) = 0;
};

/** Wraps AccelStepper for enable pin semantics used by this fixture. */
template<typename StepperT>
struct StepperActuator final : IActuator {
  StepperT&    stepper;
  const int    enablePin;
  StepperActuator(StepperT& s, int enPin) : stepper(s), enablePin(enPin) {}
  void setEnabled(bool on) override {
    if (enablePin < 0) return;
    digitalWrite(enablePin, on ? LOW : HIGH);
  }
};

/** Wraps HX711_MP calibrated read. Caller must hold lcMutex if concurrent access. */
template<typename LcT>
struct LoadCellSensor final : ISensor {
  LcT& lc;
  explicit LoadCellSensor(LcT& l) : lc(l) {}
  float readPrimary(int samples = 1) override { return lc.get_units(samples); }
};

/*
 * ServoActuator — drives a hobby servo via the ESP32 LEDC peripheral at 50 Hz.
 *
 * Pulse width <-> angle mapping uses a configurable [minPulseUs..maxPulseUs] range
 * so each physical servo can be calibrated to its true 0..180 deg endpoints
 * without losing range (datasheets vary; common range is 500..2500 us).
 *
 * The LEDC API differs between Arduino-ESP32 v2.x and v3.x; both are supported.
 * On v2.x we use an explicit channel; on v3.x the pin IS the handle.
 *
 * setEnabled(false) stops emitting pulses (pin held LOW) — useful to silence a
 * twitchy/hot servo when you don't need to hold position.
 */
struct ServoActuator final : IActuator {
  const int      pin;
  const uint32_t freqHz;
  const uint8_t  resBits;
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3))
  const uint8_t  channel;
#endif
  bool           attached = false;
  uint16_t       lastUs   = 1500;

  ServoActuator(int p, uint32_t f, uint8_t r
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3))
                , uint8_t ch
#endif
                )
    : pin(p), freqHz(f), resBits(r)
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3))
      , channel(ch)
#endif
  {}

  bool begin() {
    if (pin < 0) return false;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    attached = ledcAttach(pin, freqHz, resBits);
#else
    ledcSetup(channel, freqHz, resBits);
    ledcAttachPin(pin, channel);
    attached = true;
#endif
    return attached;
  }

  /* Write a raw pulse width in microseconds. */
  void writeMicroseconds(uint16_t us) {
    if (!attached) return;
    const uint32_t periodUs   = 1000000UL / freqHz;
    const uint32_t maxDutyVal = (1UL << resBits) - 1UL;
    if (us > periodUs) us = periodUs;
    uint32_t duty = ((uint32_t)us * maxDutyVal) / periodUs;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
    lastUs = us;
  }

  /* Map degrees -> microseconds using calibrated end-points and write. */
  void writeDegrees(float deg, uint16_t minUs, uint16_t maxUs) {
    if (deg < 0.f) deg = 0.f;
    if (deg > 180.f) deg = 180.f;
    float us = (float)minUs + (deg / 180.f) * (float)(maxUs - minUs);
    writeMicroseconds((uint16_t)(us + 0.5f));
  }

  /* IActuator: setEnabled(false) parks the pin LOW (no pulses). */
  void setEnabled(bool on) override {
    if (!attached) return;
    if (on) {
      writeMicroseconds(lastUs);
    } else {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      ledcWrite(pin, 0);
#else
      ledcWrite(channel, 0);
#endif
    }
  }
};

/*
 * BreakBeamSensor — digital input with configurable active level.
 *
 * readPrimary() returns 1.0f when the beam is *broken*, 0.0f when clear,
 * so the value can be averaged or thresholded by callers expecting a float.
 * isBroken() / rawLevel() are the typical entry points.
 *
 * usePullup is honored only if the chosen pin supports it (input-only pins
 * 34/35/36/39 do not — caller is responsible for an external pull-up there).
 */
// Break-beam sensing mode.
//   DIGITAL : one signal wire, HIGH/LOW output (E3F-DS30C4 and other NPN/photo
//             modules). Uses digitalRead() + activeLow polarity.
//   ANALOG  : through-beam receiver whose output tracks received light level.
//             Uses analogRead() + threshold. "broken" latches with hysteresis.
enum class BeamMode : uint8_t { Digital = 0, Analog = 1 };

struct BreakBeamSensor final : ISensor {
  const int pin;
  bool      activeLow = true;   // DIGITAL: LOW == broken.
                                // ANALOG:  broken when reading < threshold.
  bool      usePullup = false;  // DIGITAL only (ignored on input-only 34/35/36/39).

  // ---- ANALOG mode configuration ----
  BeamMode  mode        = BeamMode::Digital;
  uint16_t  threshold   = 2048; // ADC trip point (0..4095, 12-bit)
  uint16_t  hysteresis  = 150;  // +/- band around threshold (0 = none)
  // Analog sampling: peak-hold over a burst. Pulsed/modulated beams read high
  // only during a pulse, so a MAX over a window spanning several pulses cleanly
  // separates "beam present" from "blocked" where a single sample or short mean
  // cannot. sampleCount * sampleGapUs sets the window (~40*200us = 8ms).
  uint8_t   sampleCount = 40;
  uint16_t  sampleGapUs = 200;

  // Latched broken-state for analog hysteresis. mutable so isBroken() stays
  // const and keeps working through the existing const& call sites.
  mutable bool m_brokenLatched = false;

  BreakBeamSensor(int p, bool actLow, bool pullup)
    : pin(p), activeLow(actLow), usePullup(pullup) {}

  void begin() {
    if (pin < 0) return;
    if (mode == BeamMode::Digital) pinMode(pin, usePullup ? INPUT_PULLUP : INPUT);
    else                           pinMode(pin, INPUT);   // analog: no pull-up
    m_brokenLatched = false;
  }

  // Peak (max) over a short burst spread across the beam's modulation period.
  // Robust to the deep downward excursions a pulsed emitter produces while the
  // beam is CLEAR -- only whether the signal *reaches* high matters.
  int readAnalogPeak() const {
    int n = (sampleCount < 1) ? 1 : sampleCount;
    int peak = 0;
    for (int i = 0; i < n; ++i) {
      int v = analogRead(pin);
      if (v > peak) peak = v;
      if (sampleGapUs) delayMicroseconds(sampleGapUs);
    }
    return peak;
  }

  // Diagnostic raw value: digital level (DIGITAL) or peak ADC (ANALOG).
  int rawLevel() const {
    if (pin < 0) return -1;
    return (mode == BeamMode::Digital) ? digitalRead(pin) : readAnalogPeak();
  }

  bool isBroken() const {
    if (pin < 0) return false;

    if (mode == BeamMode::Digital) {
      int lvl = digitalRead(pin);
      return activeLow ? (lvl == LOW) : (lvl == HIGH);
    }

    // ANALOG: compare peak light level to threshold, with a hysteresis latch.
    int v  = readAnalogPeak();
    int hi = (int)threshold + (int)hysteresis;
    int lo = (int)threshold - (int)hysteresis;
    if (activeLow) {                 // broken == beam interrupted == low reading
      if      (v <= lo) m_brokenLatched = true;
      else if (v >= hi) m_brokenLatched = false;
    } else {                         // inverted receiver: broken == high reading
      if      (v >= hi) m_brokenLatched = true;
      else if (v <= lo) m_brokenLatched = false;
    }
    return m_brokenLatched;
  }

  float readPrimary(int /*samples*/ = 1) override {
    return isBroken() ? 1.0f : 0.0f;
  }
};

inline void linkAnnounceProtocol(HardwareSerial& uart, uint8_t ver) {
  uart.printf("proto=%u\n", ver);
}

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"

// ─── RC ───────────────────────────────────────────────────────────────────────
// Decodes a 6-channel PPM stream (FlySky FS-iA6B) via a falling-edge GPIO ISR.
//
// Timing model: falling-to-falling interval = channel pulse width (1000–2000 µs).
// An interval > PPM_SYNC_US marks the frame boundary; the next PPM_CHANNELS
// intervals populate ch[0..5].

class RC {
public:
    static void begin(uint8_t pin);

    // Returns true and copies a NEW frame, consuming the freshness flag.
    // Use from the control task (each frame processed exactly once).
    static bool getFrame(PPMFrame& out);

    // Copies the latest frame WITHOUT consuming freshness (safe for telemetry).
    static void peekFrame(PPMFrame& out);

    // True if a valid frame arrived within PPM_TIMEOUT_MS.
    static bool isConnected();

    // Runtime PPM calibration (per channel min/neutral/max + deadband).
    static void setCalib(const PpmCalibPayload& p);

    // Normalise a channel's raw µs to [-1,+1] using its calibration.
    static float normalise(uint8_t channel, uint16_t raw_us);

private:
    static void isr();   // IRAM_ATTR on definition
};

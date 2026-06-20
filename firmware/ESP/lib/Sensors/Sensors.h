#pragma once
#include <stdint.h>
#include "robot_types.h"

// ─── Sensors ──────────────────────────────────────────────────────────────────
// LIS3MDL magnetometer over I2C (heading reference). RoboCorea has no IMU on the
// ESP32 — orientation comes from the ZED2 stereo camera on the Jetson.
//
// The magnetometer stays idle until enabled via setEnabledMask() (the GUI
// controls this on /sensors/enable_mask). The thermal camera (MLX90640) and any
// gas sensor are NOT handled here on RoboCorea — thermal lives on the Jetson.
//
// Call begin() once, then runOnce() repeatedly from the sensor task.

class Sensors {
public:
    static bool begin();

    // Rate-limited internally. Returns the bitmask of sensors read this tick.
    static uint8_t runOnce();

    static void    setEnabledMask(uint8_t mask);
    static uint8_t getEnabledMask();

    static void getMag(MagData& out);

private:
    static void readMag();

    static uint8_t s_mask;
    static MagData s_mag;
    static bool    s_mag_ok;
};

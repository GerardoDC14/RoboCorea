#pragma once
#include <stdint.h>
#include "robot_types.h"

// ─── Sensors ──────────────────────────────────────────────────────────────────
// BNO055 9-DOF IMU + LIS3MDL magnetometer over I2C.
//
// Both stay idle until enabled via setEnabledMask() (the GUI controls this on
// /sensors/enable_mask). The thermal camera (MLX90640) and any gas sensor are
// NOT handled here on RoboCorea — thermal lives on the Jetson.
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
    static void getImu(ImuData& out);

private:
    static void readMag();
    static void readImu();

    static uint8_t s_mask;
    static MagData s_mag;
    static ImuData s_imu;
    static bool    s_mag_ok;
    static bool    s_bno_ok;
};

#include "Locomotion.h"
#include "CANInterface.h"
#include "config.h"
#include <cmath>

float Locomotion::s_track_left  = 0.0f;
float Locomotion::s_track_right = 0.0f;
float Locomotion::s_flip[4]     = { 0, 0, 0, 0 };

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void Locomotion::begin() {
    s_track_left = s_track_right = 0.0f;
    CANInterface::sendTrackSpeeds(0.0f, 0.0f);
    // Coast the flippers until the control loop deliberately enables them.
    CANInterface::sendFlipperAngles(s_flip, /*enabled=*/false);
}

void Locomotion::setTrackSpeeds(float left_norm, float right_norm) {
    s_track_left  = clampf(left_norm,  -1.0f, 1.0f);
    s_track_right = clampf(right_norm, -1.0f, 1.0f);
    CANInterface::sendTrackSpeeds(s_track_left, s_track_right);
}

void Locomotion::setDriveCommand(float forward, float turn) {
    float left  = forward + turn;
    float right = forward - turn;
    float mag = fmaxf(fabsf(left), fabsf(right));
    if (mag > 1.0f) { left /= mag; right /= mag; }   // preserve ratio, clamp magnitude
    setTrackSpeeds(left, right);
}

void Locomotion::setFlipperAngles(const float deg[4], bool enabled) {
    for (int i = 0; i < 4; i++) s_flip[i] = deg[i];
    CANInterface::sendFlipperAngles(s_flip, enabled);
}

void Locomotion::neutralise() {
    s_track_left = s_track_right = 0.0f;
    CANInterface::sendTrackSpeeds(0.0f, 0.0f);
    // Failsafe = HOLD the flippers where they are (never drive them home).
    CANInterface::sendFlipperAngles(s_flip, /*enabled=*/true);
}

void Locomotion::estopOutputs() {
    s_track_left = s_track_right = 0.0f;
    CANInterface::sendTrackSpeeds(0.0f, 0.0f);
    CANInterface::sendFlipperAngles(s_flip, /*enabled=*/FLIPPER_ESTOP_HOLD ? true : false);
}

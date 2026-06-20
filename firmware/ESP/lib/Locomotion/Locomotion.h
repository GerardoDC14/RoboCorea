#pragma once
#include <stdint.h>

// ─── Locomotion ───────────────────────────────────────────────────────────────
// Output layer over the drivetrain VESCs (CAN). Forwards commands to CANInterface.
//
//   • Tracks:   differential mix of forward/turn → left/right velocity ([-1,+1]).
//   • Flippers: absolute target angles [0,360) — the position loop lives ON the
//     VESC (LispBM). We remember the last targets so we can re-send them to HOLD
//     (failsafe) or send enable=false to COAST (e-stop).

class Locomotion {
public:
    static void begin();                                       // tracks 0, flippers coast

    static void setTrackSpeeds(float left_norm, float right_norm);
    static void setDriveCommand(float forward, float turn);    // mixes to L/R
    static void setFlipperAngles(const float deg[4], bool enabled);  // FL,FR,RL,RR

    static void neutralise();      // tracks 0, flippers HOLD last target (RC-loss/standby)
    static void estopOutputs();    // tracks 0, flippers hold or coast per FLIPPER_ESTOP_HOLD

    static float getTrackLeft()  { return s_track_left; }
    static float getTrackRight() { return s_track_right; }

private:
    static float s_track_left;
    static float s_track_right;
    static float s_flip[4];        // last commanded flipper angles (for hold)
};

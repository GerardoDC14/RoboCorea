#pragma once
#include "robot_types.h"

// ─── Control ──────────────────────────────────────────────────────────────────
// Top-level state machine. Runs on Core 1 at PRIO_CONTROL (CONTROL_LOOP_HZ).
//
//  INIT ──(hw ready)──► STANDBY ──(PPM link)──► active mode (follows Ch5)
//
//  Ch5 (3-position lever) selects a keybind row; each row maps Ch1–4,6 to a
//  ChannelFunction. The row received from the PC (MSG_KEYBIND) replaces the
//  default. The high-level RobotMode (NORMAL / FLIPPER / ARM) is derived from
//  what is bound in the active row.
//
//  Ch6 HIGH or an MSG_ESTOP frame → ESTOP. Cleared by Ch6 LOW and ESTOP_CLEAR.

class Control {
public:
    static void begin();
    static void tick();

    // Setters invoked from comms callbacks / other tasks.
    static void triggerEstop();
    static void clearEstop();
    static void setArmJoints(const ArmJointsPayload& payload);
    static void setSensorMask(uint8_t mask);
    static void setKeybind(const KeybindPayload& payload);

    static RobotMode getMode();
    static void      getSystemStatus(SystemStatus& out);

private:
    static void applyKeybindRow(int mode_idx, const PPMFrame& ppm);
    static int  decodeModeIndex(const PPMFrame& ppm);

    static RobotMode    s_mode;
    static ArmJoints    s_arm_joints;
    static uint8_t      s_sensor_mask;
    static KeybindTable s_keybind;
    static bool         s_hw_estop;       // Ch6 hardware e-stop
    static bool         s_virtual_estop;  // PC/comms e-stop
    static float        s_deadband;

    // Flippers: position loop runs on the VESC. We integrate the stick into a
    // per-flipper target angle (FL,FR,RL,RR) and send it; HOLD when unbound.
    static float        s_flip_target[4];
    static bool         s_flip_seeded;    // false → re-seed from measured (bumpless)
};

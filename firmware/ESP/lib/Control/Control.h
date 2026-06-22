#pragma once
#include "robot_types.h"

struct PPMFrame;

// ─── Control ──────────────────────────────────────────────────────────────────
// Top-level state machine. Runs on Core 1 at PRIO_CONTROL (CONTROL_LOOP_HZ).
//
//  INIT ──(hw ready)──► STANDBY ──(PPM link)──► NORMAL (drive + flippers)
//
// FIXED RC control scheme (no keybind table — see config.h "Channel roles"):
//   Ch1  flipper L/R selector (min=left, center=both, max=right)
//   Ch2  flipper rate (drives the flipper(s) Ch1+Ch5 select)
//   Ch3  traction forward / back        Ch4  traction turn
//   Ch5  2-state pair select (max=FRONT FL/FR, min=REAR RL/RR)
//   Ch6  3-position lever (down=E-STOP, center=normal, up=virtual-flip)
// Drive + flippers are always active together; arm joints arrive from the
// workstation (MSG_ARM_JOINTS) and are relayed whenever armed & not e-stopped.
//
// Ch6-down or an MSG_ESTOP frame → ESTOP. Cleared by Ch6-center + ESTOP_CLEAR.

class Control {
public:
    static void begin();
    static void tick();

    // Setters invoked from comms callbacks / other tasks.
    static void triggerEstop();
    static void clearEstop();
    static void setArmJoints(const ArmJointsPayload& payload);
    static void setSensorMask(uint8_t mask);

    static RobotMode getMode();
    static void      getSystemStatus(SystemStatus& out);

private:
    // The fixed control mapping (Ch1-6 → traction + flippers, with virtual flip).
    static void applyControl(const PPMFrame& ppm);

    static RobotMode    s_mode;
    static ArmJoints    s_arm_joints;
    static uint8_t      s_sensor_mask;
    static bool         s_hw_estop;       // Ch6-down RC e-stop
    static bool         s_virtual_estop;  // PC/comms e-stop
    static bool         s_virtual_flip;   // Ch6-up "drive from the other end"
    static float        s_deadband;

    // Flippers: position loop runs on the VESC. We integrate the stick into a
    // per-flipper target angle (FL,FR,RL,RR) and send it; HOLD when stick centered.
    static float        s_flip_target[4];
    static bool         s_flip_seeded;    // false → re-seed from measured (bumpless)
};

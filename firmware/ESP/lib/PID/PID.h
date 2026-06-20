#pragma once

// ─── PID Controller ──────────────────────────────────────────────────────────
// Reusable PID with two update modes:
//   update()        — linear error, derivative-on-measurement (no setpoint kick)
//   updateAngular() — shortest-path angular error, derivative-on-error
//
// Features: D-term low-pass filter, conditional-integration anti-windup,
// per-instance output clamping, full state inspection.

class PID {
public:
    void configure(float kp, float ki, float kd,
                   float i_max   = 10.0f,
                   float out_min = -1.0f, float out_max = 1.0f,
                   float d_alpha = 0.0f);

    // Linear PID. Derivative on measurement (avoids derivative kick).
    float update(float setpoint, float measurement, float dt);

    // Angular PID. Error is the shortest path in [-180°, +180°];
    // derivative on error (measurement wraps at the 0/360 boundary).
    float updateAngular(float setpoint_deg, float measurement_deg, float dt);

    void reset();
    void setGains(float kp, float ki, float kd);

    float error()    const { return m_error; }
    float integral() const { return m_integral; }
    float pTerm()    const { return m_p_term; }
    float iTerm()    const { return m_i_term; }
    float dTerm()    const { return m_d_term; }
    float output()   const { return m_output; }

private:
    float m_kp = 0, m_ki = 0, m_kd = 0;
    float m_i_max   = 10.0f;
    float m_out_min = -1.0f, m_out_max = 1.0f;
    float m_d_alpha = 0.0f;

    float m_error      = 0;
    float m_integral   = 0;
    float m_prev_meas  = 0;
    float m_prev_err   = 0;
    float m_d_filtered = 0;
    float m_p_term = 0, m_i_term = 0, m_d_term = 0;
    float m_output = 0;
    bool  m_first  = true;
};

#pragma once

#include <QFrame>
#include <QLabel>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <atomic>

class QTextEdit;
class SpeechProcessor;

// Right-column dashboard: connection/heartbeat + uptime, magnetometer and IMU
// readouts with per-sensor enable toggles, the software e-stop, and the speech
// transcription panel + audio-monitor toggle.
//
// RoboCorea changes vs legacy: no gas sensor. Audio is the Opus track demuxed
// from the C920 A/V stream (fed to the SpeechProcessor by MainWindow), not a ROS
// topic. Sensor-enable mask bits: bit0 mag, bit1 thermal (driven by the video
// panel's thermal selection, not a button), bit3 imu.
class DashboardPanel : public QWidget {
    Q_OBJECT
public:
    explicit DashboardPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    // Vosk transcriber owned by the dashboard; MainWindow routes the A/V stream's
    // decoded PCM into it via SpeechProcessor::pushAudio().
    SpeechProcessor* speechProcessor() const { return speech_processor_; }

signals:
    void resetSourcesRequested();
    void settingsRequested();
    void audioMonitorToggled(bool enabled);  // → GstAvStream playback
    void magnetometerUpdated(double x, double y, double z);
    void imuUpdated(double yaw, double pitch, double roll);
    void telemetryReceived();   // heartbeat
    void uptimeUpdated(float uptime_s);

public slots:
    // Called by VideoPanel when any widget selects/deselects the thermal source.
    void setThermalEnabled(bool enabled);

private slots:
    void onEstopToggled(bool checked);
    void onAudioToggled(bool checked);
    void onTranscriptionUpdated(const QString& text);
    void onMagnetometerUpdated(double x, double y, double z);
    void onImuUpdated(double yaw, double pitch, double roll);
    void onTelemetryReceived();
    void onHeartbeatCheck();
    void onUptimeUpdated(float uptime_s);
    void onClearAll();
    void publishEstopState();
    void onSensorToggled();

private:
    void setConnState(const QString& color, const QString& label);
    void publishSensorMask();

    rclcpp::Node::SharedPtr node_;

    // Connection status
    QLabel*             conn_indicator_;
    QLabel*             conn_label_;
    QLabel*             uptime_label_;
    QTimer*             heartbeat_timer_;
    QPropertyAnimation* pulse_anim_{nullptr};
    bool                hb_received_{false};
    int                 hb_miss_count_{3};
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telemetry_sub_;

    // Magnetometer
    QLabel* mag_x_;
    QLabel* mag_y_;
    QLabel* mag_z_;

    // IMU orientation
    QLabel* imu_yaw_;
    QLabel* imu_pitch_;
    QLabel* imu_roll_;

    // Speech / audio monitor
    QTextEdit*       transcription_{nullptr};
    QPushButton*     audio_btn_{nullptr};
    SpeechProcessor* speech_processor_{nullptr};

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Sensor enable mask toggles (bit0=mag, bit1=thermal, bit3=imu).
    QPushButton* mag_toggle_;
    QPushButton* imu_toggle_;
    uint8_t      sensor_mask_{0};
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr sensor_mask_pub_;

    // Controls
    QPushButton* clear_btn_;
    QPushButton* reset_btn_;
    QPushButton* settings_btn_;
    QPushButton* estop_btn_;

    // E-STOP (republished ~10 Hz)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    QTimer*           estop_timer_;
    std::atomic<bool> estop_active_{false};
};

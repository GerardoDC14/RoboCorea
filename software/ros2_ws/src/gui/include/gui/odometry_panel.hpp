#pragma once

#include <QFrame>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <array>

// Odometry panel (bottom-left of the right section, beside the dashboard):
// mode/flags, track RPM, the integrated track (wheel) odometry from the VESC
// tachometers, the four flipper angles, the six-VESC telemetry table, and
// arm-joint telemetry (ODrive J1–3 / ZE300 J4 / LKTech J5–6). RoboCorea is
// single-robot, so there is no robot-type switch and no Jaguar PWM "main motor".
class OdometryPanel : public QWidget {
    Q_OBJECT
public:
    explicit OdometryPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

signals:
    // ROS thread → Qt main thread bridges.
    void tracksUpdated(double left_rpm, double right_rpm);
    void wheelOdomUpdated(double x_m, double y_m, double yaw_deg, double vx);
    void flipperExtUpdated(float fl, float fr, float rl, float rr);
    void modeUpdated(const QString& mode);
    void flagsUpdated(int flags);
    void vescStatusUpdated(int id, float erpm, float current, float duty,
                           float temp_fet, float temp_motor);
    void armJointUpdated(int joint, double angle_deg, double secondary,
                         const QString& sec_unit);

private slots:
    void onTracksUpdated(double left_rpm, double right_rpm);
    void onWheelOdomUpdated(double x_m, double y_m, double yaw_deg, double vx);
    void onFlipperExtUpdated(float fl, float fr, float rl, float rr);
    void onModeUpdated(const QString& mode);
    void onFlagsUpdated(int flags);
    void onVescStatusUpdated(int id, float erpm, float current, float duty,
                             float temp_fet, float temp_motor);
    void onArmJointUpdated(int joint, double angle_deg, double secondary,
                           const QString& sec_unit);

private:
    void buildLayout();
    QLabel* makeValueLabel(const QString& initial = "--");
    QLabel* makeHeaderLabel(const QString& text);
    QLabel* makeAxisLabel(const QString& text);
    QFrame* makeHSep();

    rclcpp::Node::SharedPtr node_;
    QVBoxLayout* main_layout_{nullptr};

    // Status
    QLabel* mode_label_;
    QLabel* flags_label_;

    // Traction
    QLabel* trac_left_rpm_;
    QLabel* trac_right_rpm_;

    // Track (wheel) odometry from the VESC tachometers
    QLabel* odom_x_;
    QLabel* odom_y_;
    QLabel* odom_yaw_;
    QLabel* odom_vx_;

    // Flippers
    QLabel* flip_fl_;
    QLabel* flip_fr_;
    QLabel* flip_rl_;
    QLabel* flip_rr_;

    // VESC table (IDs 1-6; index 0 unused)
    struct VescRow {
        QLabel* erpm;
        QLabel* current;
        QLabel* duty;
        QLabel* temp_fet;
        QLabel* temp_motor;
    };
    std::array<VescRow, 7> vesc_rows_{};

    // Arm table (J1-J6; index 0 unused)
    struct ArmRow {
        QLabel* angle;
        QLabel* secondary;
    };
    std::array<ArmRow, 7> arm_rows_{};

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr tracks_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr flipper_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flags_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vesc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odrive_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ze300_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lktech_sub_;
};

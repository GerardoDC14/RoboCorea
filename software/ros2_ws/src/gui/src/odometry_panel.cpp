#include "gui/odometry_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>

#include <cmath>

// VESC IDs per architecture.md §8.1 (traction L/R = 1/2, flippers FL/FR/RL/RR = 3-6).
static const char* VESC_NAMES[7] = {"?", "TL", "TR", "FL", "FR", "RL", "RR"};
static const char* ARM_NAMES[7]  = {"?", "J1", "J2", "J3", "J4", "J5", "J6"};

OdometryPanel::OdometryPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    setStyleSheet("background-color: #1e1e2e;");
    buildLayout();

    connect(this, &OdometryPanel::tracksUpdated,
            this, &OdometryPanel::onTracksUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::wheelOdomUpdated,
            this, &OdometryPanel::onWheelOdomUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::flipperExtUpdated,
            this, &OdometryPanel::onFlipperExtUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::modeUpdated,
            this, &OdometryPanel::onModeUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::flagsUpdated,
            this, &OdometryPanel::onFlagsUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::vescStatusUpdated,
            this, &OdometryPanel::onVescStatusUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::armJointUpdated,
            this, &OdometryPanel::onArmJointUpdated, Qt::QueuedConnection);

    auto qos = rclcpp::QoS(10).best_effort();

    tracks_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/encoders/tracks", qos,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            emit tracksUpdated(msg->x, msg->y);
        });

    // Track (wheel) odometry integrated from the VESC tachometers on the bridge.
    wheel_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/wheel", qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto& q = msg->pose.pose.orientation;
            double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 180.0 / M_PI;
            emit wheelOdomUpdated(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                  yaw, msg->twist.twist.linear.x);
        });

    flipper_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/encoders/flipper", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 4)
                emit flipperExtUpdated(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
        });

    mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/mode", qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit modeUpdated(QString::fromStdString(msg->data));
        });

    flags_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "/robot/flags", qos,
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
            emit flagsUpdated(static_cast<int>(msg->data));
        });

    // VESC: [id, erpm, current_A, duty, temp_fet, temp_motor, voltage]
    vesc_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/vesc_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7)
                emit vescStatusUpdated(static_cast<int>(msg->data[0]),
                                       msg->data[1], msg->data[2], msg->data[3],
                                       msg->data[4], msg->data[5]);
        });

    // Arm telemetry → unified armJointUpdated(joint 1-6, angle°, secondary, unit).
    // NOTE: joint-index mapping is assumed from the bridge field order; verify
    // against the firmware once the arm is on the bench (architecture.md §8).
    // ODrive J1-3: [joint, pos_turns, vel, iq_A, busV, busA]
    odrive_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/odrive_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 6) {
                int joint = static_cast<int>(msg->data[0]) + 1;  // 0-based → J1..J3
                emit armJointUpdated(joint, msg->data[1] * 360.0, msg->data[3], "A");
            }
        });
    // ZE300 J4: [id, temp_C, iq_A, rpm, single_turn, pos_counts, out_deg]
    ze300_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/ze300_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7)
                emit armJointUpdated(4, msg->data[6], msg->data[1], "°C");
        });
    // LKTech J5-6: [joint, motor_id, temp_C, iq_A, dps, angle, out_deg]
    lktech_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/lktech_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7) {
                int motor_id = static_cast<int>(msg->data[1]);
                int joint = (motor_id == 15) ? 6 : 5;  // 14→J5, 15→J6
                emit armJointUpdated(joint, msg->data[6], msg->data[2], "°C");
            }
        });
}

void OdometryPanel::buildLayout()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(8, 6, 8, 6);
    main_layout_->setSpacing(4);

    // ── Status ───────────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Telemetry Status"));
    auto* status_grid = new QGridLayout();
    status_grid->setSpacing(3);
    status_grid->addWidget(makeAxisLabel("Mode"), 0, 0);
    mode_label_ = makeValueLabel("--");
    status_grid->addWidget(mode_label_, 0, 1);
    status_grid->addWidget(makeAxisLabel("Flags"), 1, 0);
    flags_label_ = makeValueLabel("--");
    status_grid->addWidget(flags_label_, 1, 1);
    main_layout_->addLayout(status_grid);

    main_layout_->addWidget(makeHSep());

    // ── Traction ─────────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Traction (RPM)"));
    auto* trac_row = new QHBoxLayout();
    trac_row->setSpacing(8);
    {
        auto* c = new QVBoxLayout(); c->setSpacing(1);
        c->addWidget(makeAxisLabel("Left"));
        trac_left_rpm_ = makeValueLabel("--");
        c->addWidget(trac_left_rpm_);
        trac_row->addLayout(c);
    }
    {
        auto* c = new QVBoxLayout(); c->setSpacing(1);
        c->addWidget(makeAxisLabel("Right"));
        trac_right_rpm_ = makeValueLabel("--");
        c->addWidget(trac_right_rpm_);
        trac_row->addLayout(c);
    }
    main_layout_->addLayout(trac_row);

    main_layout_->addWidget(makeHSep());

    // ── Track odometry (wheel, from the VESC tachometers) ─────────────────────
    main_layout_->addWidget(makeHeaderLabel("Track Odometry"));
    {
        auto* grid = new QGridLayout();
        grid->setSpacing(3);
        grid->addWidget(makeAxisLabel("X (m)"), 0, 0);
        odom_x_ = makeValueLabel("--"); grid->addWidget(odom_x_, 0, 1);
        grid->addWidget(makeAxisLabel("Y (m)"), 0, 2);
        odom_y_ = makeValueLabel("--"); grid->addWidget(odom_y_, 0, 3);
        grid->addWidget(makeAxisLabel("Yaw"), 1, 0);
        odom_yaw_ = makeValueLabel("--"); grid->addWidget(odom_yaw_, 1, 1);
        grid->addWidget(makeAxisLabel("vx (m/s)"), 1, 2);
        odom_vx_ = makeValueLabel("--"); grid->addWidget(odom_vx_, 1, 3);
        main_layout_->addLayout(grid);
    }

    main_layout_->addWidget(makeHSep());

    // ── Flippers ─────────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Flippers"));
    {
        auto* grid = new QGridLayout();
        grid->setSpacing(3);
        grid->addWidget(makeAxisLabel("FL"), 0, 0);
        flip_fl_ = makeValueLabel("--"); grid->addWidget(flip_fl_, 0, 1);
        grid->addWidget(makeAxisLabel("FR"), 0, 2);
        flip_fr_ = makeValueLabel("--"); grid->addWidget(flip_fr_, 0, 3);
        grid->addWidget(makeAxisLabel("RL"), 1, 0);
        flip_rl_ = makeValueLabel("--"); grid->addWidget(flip_rl_, 1, 1);
        grid->addWidget(makeAxisLabel("RR"), 1, 2);
        flip_rr_ = makeValueLabel("--"); grid->addWidget(flip_rr_, 1, 3);
        main_layout_->addLayout(grid);
    }

    main_layout_->addWidget(makeHSep());

    // ── VESC telemetry table ─────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Motor Telemetry (VESC)"));
    {
        auto* hdr = new QGridLayout();
        hdr->setSpacing(2);
        hdr->addWidget(makeAxisLabel("ID"),   0, 0);
        hdr->addWidget(makeAxisLabel("eRPM"), 0, 1);
        hdr->addWidget(makeAxisLabel("A"),    0, 2);
        hdr->addWidget(makeAxisLabel("Duty"), 0, 3);
        hdr->addWidget(makeAxisLabel("Tfet"), 0, 4);
        hdr->addWidget(makeAxisLabel("Tmot"), 0, 5);
        main_layout_->addLayout(hdr);

        for (int id = 1; id <= 6; ++id) {
            auto* row = new QGridLayout();
            row->setSpacing(2);
            row->addWidget(makeAxisLabel(VESC_NAMES[id]), 0, 0);
            vesc_rows_[id].erpm       = makeValueLabel("--");
            vesc_rows_[id].current    = makeValueLabel("--");
            vesc_rows_[id].duty       = makeValueLabel("--");
            vesc_rows_[id].temp_fet   = makeValueLabel("--");
            vesc_rows_[id].temp_motor = makeValueLabel("--");
            for (QLabel* l : {vesc_rows_[id].erpm, vesc_rows_[id].current,
                              vesc_rows_[id].duty, vesc_rows_[id].temp_fet,
                              vesc_rows_[id].temp_motor})
                l->setStyleSheet("color: #4fc3f7; font-size: 10px;");
            row->addWidget(vesc_rows_[id].erpm,       0, 1);
            row->addWidget(vesc_rows_[id].current,    0, 2);
            row->addWidget(vesc_rows_[id].duty,       0, 3);
            row->addWidget(vesc_rows_[id].temp_fet,   0, 4);
            row->addWidget(vesc_rows_[id].temp_motor, 0, 5);
            main_layout_->addLayout(row);
        }
    }

    main_layout_->addWidget(makeHSep());

    // ── Arm telemetry ────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Arm (ODrive / ZE300 / LKTech)"));
    {
        auto* hdr = new QGridLayout();
        hdr->setSpacing(2);
        hdr->addWidget(makeAxisLabel("Joint"), 0, 0);
        hdr->addWidget(makeAxisLabel("Angle"), 0, 1);
        hdr->addWidget(makeAxisLabel("Iq/T"),  0, 2);
        main_layout_->addLayout(hdr);

        for (int j = 1; j <= 6; ++j) {
            auto* row = new QGridLayout();
            row->setSpacing(2);
            row->addWidget(makeAxisLabel(ARM_NAMES[j]), 0, 0);
            arm_rows_[j].angle     = makeValueLabel("--");
            arm_rows_[j].secondary = makeValueLabel("--");
            for (QLabel* l : {arm_rows_[j].angle, arm_rows_[j].secondary})
                l->setStyleSheet("color: #4fc3f7; font-size: 10px;");
            row->addWidget(arm_rows_[j].angle,     0, 1);
            row->addWidget(arm_rows_[j].secondary, 0, 2);
            main_layout_->addLayout(row);
        }
    }

    main_layout_->addStretch();
}

QLabel* OdometryPanel::makeValueLabel(const QString& initial)
{
    auto* l = new QLabel(initial, this);
    l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    l->setStyleSheet("color: #4fc3f7; font-size: 13px;");
    return l;
}

QLabel* OdometryPanel::makeHeaderLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #aaa; font-weight: bold;");
    return l;
}

QLabel* OdometryPanel::makeAxisLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #888;");
    return l;
}

QFrame* OdometryPanel::makeHSep()
{
    auto* s = new QFrame(this);
    s->setFrameShape(QFrame::HLine);
    s->setStyleSheet("color: #444;");
    return s;
}

void OdometryPanel::onTracksUpdated(double left_rpm, double right_rpm)
{
    trac_left_rpm_->setText(QString::number(left_rpm, 'f', 1));
    trac_right_rpm_->setText(QString::number(right_rpm, 'f', 1));
}

void OdometryPanel::onWheelOdomUpdated(double x_m, double y_m, double yaw_deg, double vx)
{
    odom_x_->setText(QString::number(x_m, 'f', 2));
    odom_y_->setText(QString::number(y_m, 'f', 2));
    odom_yaw_->setText(QString::number(yaw_deg, 'f', 1) + "°");
    odom_vx_->setText(QString::number(vx, 'f', 2));
}

void OdometryPanel::onFlipperExtUpdated(float fl, float fr, float rl, float rr)
{
    flip_fl_->setText(QString::number(fl, 'f', 1) + "°");
    flip_fr_->setText(QString::number(fr, 'f', 1) + "°");
    flip_rl_->setText(QString::number(rl, 'f', 1) + "°");
    flip_rr_->setText(QString::number(rr, 'f', 1) + "°");
}

void OdometryPanel::onModeUpdated(const QString& mode)
{
    mode_label_->setText(mode);
    if (mode == "ESTOP")
        mode_label_->setStyleSheet("color: #cc3333; font-size: 12px; font-weight: bold;");
    else if (mode == "STANDBY")
        mode_label_->setStyleSheet("color: #ccaa00; font-size: 12px;");
    else if (mode == "NORMAL" || mode == "FLIPPER")
        mode_label_->setStyleSheet("color: #33cc33; font-size: 12px;");
    else
        mode_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
}

void OdometryPanel::onFlagsUpdated(int flags)
{
    QStringList parts;
    if (flags & 0x01) parts << "PPM";
    if (flags & 0x02) parts << "SENS";
    if (flags & 0x04) parts << "CAN";
    if (flags & 0x08) parts << "ESTOP";
    flags_label_->setText(parts.isEmpty() ? "none" : parts.join(" | "));
}

void OdometryPanel::onVescStatusUpdated(int id, float erpm, float current, float duty,
                                        float temp_fet, float temp_motor)
{
    if (id < 1 || id > 6) return;
    auto& row = vesc_rows_[id];
    row.erpm->setText(QString::number(static_cast<int>(erpm)));
    row.current->setText(QString::number(current, 'f', 1) + "A");
    row.duty->setText(QString::number(duty * 100.0f, 'f', 0) + "%");
    row.temp_fet->setText(QString::number(temp_fet, 'f', 0) + "°");
    row.temp_motor->setText(QString::number(temp_motor, 'f', 0) + "°");
}

void OdometryPanel::onArmJointUpdated(int joint, double angle_deg, double secondary,
                                      const QString& sec_unit)
{
    if (joint < 1 || joint > 6) return;
    arm_rows_[joint].angle->setText(QString::number(angle_deg, 'f', 1) + "°");
    arm_rows_[joint].secondary->setText(QString::number(secondary, 'f', 1) + sec_unit);
}

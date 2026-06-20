#include "gui/dashboard_panel.hpp"
#include "gui/speech_processor.hpp"
#include "gui/app_settings.hpp"

#include <QFont>
#include <QGraphicsOpacityEffect>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QTextEdit>
#include <QVBoxLayout>

#include <cmath>

DashboardPanel::DashboardPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    const QString hdr_style = "color: #aaa; font-weight: bold;";
    const QString lbl_style = "color: #888;";
    const QString val_style = "color: #4fc3f7; font-size: 13px;";

    // ── Connection status ────────────────────────────────────────────────────
    auto* conn_hdr = new QLabel("Connection Status", this);
    conn_hdr->setAlignment(Qt::AlignHCenter);
    conn_hdr->setStyleSheet(hdr_style);
    layout->addWidget(conn_hdr);

    auto* conn_row = new QHBoxLayout();
    conn_row->setSpacing(6);
    conn_indicator_ = new QLabel("●", this);
    conn_indicator_->setStyleSheet("color: #cc3333; font-size: 14px;");
    conn_label_ = new QLabel("Offline", this);
    conn_label_->setStyleSheet(lbl_style);
    uptime_label_ = new QLabel("--", this);
    uptime_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
    conn_row->addStretch();
    conn_row->addWidget(conn_indicator_);
    conn_row->addWidget(conn_label_);
    conn_row->addWidget(uptime_label_);
    conn_row->addStretch();
    layout->addLayout(conn_row);

    auto* opacity = new QGraphicsOpacityEffect(conn_indicator_);
    conn_indicator_->setGraphicsEffect(opacity);
    pulse_anim_ = new QPropertyAnimation(opacity, "opacity", this);
    pulse_anim_->setDuration(800);
    pulse_anim_->setKeyValueAt(0.0, 1.0);
    pulse_anim_->setKeyValueAt(0.4, 0.5);
    pulse_anim_->setKeyValueAt(1.0, 1.0);
    pulse_anim_->setEasingCurve(QEasingCurve::InOutSine);

    auto add_hsep = [&]() {
        auto* s = new QFrame(this);
        s->setFrameShape(QFrame::HLine);
        s->setStyleSheet("color: #444;");
        layout->addWidget(s);
    };
    add_hsep();

    // ── Helpers ──────────────────────────────────────────────────────────────
    auto make_val = [&]() {
        auto* l = new QLabel("--", this);
        l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        l->setStyleSheet(val_style);
        return l;
    };
    auto make_axis = [&](const char* text) {
        auto* l = new QLabel(text, this);
        l->setAlignment(Qt::AlignHCenter);
        l->setStyleSheet(lbl_style);
        return l;
    };
    auto make_sensor_toggle = [&]() {
        auto* btn = new QPushButton("OFF", this);
        btn->setCheckable(true);
        btn->setFixedSize(36, 18);
        QFont f = btn->font();
        f.setPointSize(7);
        btn->setFont(f);
        btn->setStyleSheet(
            "QPushButton { background-color: #3a2a2a; color: #888; padding: 1px; "
            "border: 1px solid #5a3a3a; border-radius: 3px; }"
            "QPushButton:hover { background-color: #4a3030; }"
            "QPushButton:checked { background-color: #1a5a1a; color: #8afa8a; "
            "border-color: #2a8a2a; }");
        return btn;
    };

    // ── Magnetometer ─────────────────────────────────────────────────────────
    {
        auto* mag_hdr_row = new QHBoxLayout();
        mag_hdr_row->setSpacing(4);
        auto* mag_hdr = new QLabel("Magnetometer (µT)", this);
        mag_hdr->setStyleSheet(hdr_style);
        mag_toggle_ = make_sensor_toggle();
        connect(mag_toggle_, &QPushButton::toggled, this, &DashboardPanel::onSensorToggled);
        mag_hdr_row->addWidget(mag_hdr, 1);
        mag_hdr_row->addWidget(mag_toggle_);
        layout->addLayout(mag_hdr_row);
    }
    mag_x_ = make_val(); mag_y_ = make_val(); mag_z_ = make_val();
    for (auto [lbl, val] : {std::pair{"X", mag_x_}, {"Y", mag_y_}, {"Z", mag_z_}}) {
        auto* row = new QHBoxLayout();
        row->addWidget(make_axis(lbl));
        row->addWidget(val, 1);
        layout->addLayout(row);
    }

    add_hsep();

    // ── Orientation (from the ZED2 IMU; no ESP32 IMU) ─────────────────────────
    // Sourced from the ZED2 camera driver, not the ESP32 — so there is no
    // sensor-enable toggle here (the ZED isn't controlled by the enable mask).
    {
        auto* imu_hdr = new QLabel("Orientation (ZED2)", this);
        imu_hdr->setStyleSheet(hdr_style);
        layout->addWidget(imu_hdr);
    }
    auto* imu_row = new QHBoxLayout();
    imu_row->setSpacing(8);
    imu_yaw_ = make_val(); imu_pitch_ = make_val(); imu_roll_ = make_val();
    for (auto [lbl, val] : {std::pair{"Yaw", imu_yaw_}, {"Pitch", imu_pitch_}, {"Roll", imu_roll_}}) {
        auto* col = new QVBoxLayout();
        col->setSpacing(1);
        col->addWidget(make_axis(lbl));
        col->addWidget(val);
        imu_row->addLayout(col);
    }
    layout->addLayout(imu_row);

    add_hsep();

    // ── Transcription + audio monitor ────────────────────────────────────────
    {
        auto* trans_hdr_row = new QHBoxLayout();
        trans_hdr_row->setSpacing(4);
        auto* trans_label = new QLabel("Transcription", this);
        trans_label->setStyleSheet(hdr_style);
        audio_btn_ = make_sensor_toggle();
        connect(audio_btn_, &QPushButton::toggled, this, &DashboardPanel::onAudioToggled);
        trans_hdr_row->addWidget(trans_label);
        trans_hdr_row->addStretch();
        trans_hdr_row->addWidget(audio_btn_);
        layout->addLayout(trans_hdr_row);
    }
    transcription_ = new QTextEdit(this);
    transcription_->setReadOnly(true);
    transcription_->setMaximumHeight(75);
    transcription_->setStyleSheet(
        "background-color: #1a1a2e; color: #c0c0c0; border: 1px solid #333; "
        "font-size: 14px; padding: 4px;");
    transcription_->setPlaceholderText("Waiting for audio…");
    layout->addWidget(transcription_);

    speech_processor_ = new SpeechProcessor(node_, this);
    connect(speech_processor_, &SpeechProcessor::transcriptionUpdated,
            this, &DashboardPanel::onTranscriptionUpdated, Qt::QueuedConnection);
    {
        std::lock_guard<std::mutex> lk(AppSettings::instance().strings_mutex);
        speech_processor_->setGrammar(AppSettings::instance().vosk_grammar);
    }
    audio_btn_->setChecked(AppSettings::instance().audio_start_enabled.load());

    // ── Subscriptions (ROS thread → Qt via queued signals) ───────────────────
    connect(this, &DashboardPanel::magnetometerUpdated,
            this, &DashboardPanel::onMagnetometerUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::imuUpdated,
            this, &DashboardPanel::onImuUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::telemetryReceived,
            this, &DashboardPanel::onTelemetryReceived, Qt::QueuedConnection);
    connect(this, &DashboardPanel::uptimeUpdated,
            this, &DashboardPanel::onUptimeUpdated, Qt::QueuedConnection);

    auto sensor_qos = rclcpp::QoS(10).best_effort();

    mag_sub_ = node_->create_subscription<sensor_msgs::msg::MagneticField>(
        "/sensors/mag", sensor_qos,
        [this](sensor_msgs::msg::MagneticField::SharedPtr msg) {
            emit magnetometerUpdated(msg->magnetic_field.x, msg->magnetic_field.y,
                                     msg->magnetic_field.z);
        });

    // Orientation comes from the ZED2 camera's IMU (the deferred ZED/nav stack),
    // not the ESP32. Exact topic depends on the ZED launch config; this is the
    // zed-ros2-wrapper default and stays blank until that node runs.
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/zed2/zed_node/imu/data", sensor_qos,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            double qw = msg->orientation.w, qx = msg->orientation.x;
            double qy = msg->orientation.y, qz = msg->orientation.z;

            double sinr = 2.0 * (qw * qx + qy * qz);
            double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
            double roll = std::atan2(sinr, cosr) * 180.0 / M_PI;

            double sinp = 2.0 * (qw * qy - qz * qx);
            double pitch = std::abs(sinp) >= 1.0 ? std::copysign(90.0, sinp)
                                                 : std::asin(sinp) * 180.0 / M_PI;

            double siny = 2.0 * (qw * qz + qx * qy);
            double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
            double yaw = std::atan2(siny, cosy) * 180.0 / M_PI;

            emit imuUpdated(yaw, pitch, roll);
        });

    telemetry_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/telemetry", sensor_qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            emit telemetryReceived();
            if (msg->data.size() >= 4)
                emit uptimeUpdated(msg->data[3]);
        });

    heartbeat_timer_ = new QTimer(this);
    heartbeat_timer_->setInterval(1000);
    connect(heartbeat_timer_, &QTimer::timeout, this, &DashboardPanel::onHeartbeatCheck);
    heartbeat_timer_->start();

    sensor_mask_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
        "/sensors/enable_mask", 10);
    publishSensorMask();  // start with all sensors off

    layout->addStretch();
    add_hsep();

    // ── Controls ─────────────────────────────────────────────────────────────
    auto btn_style = [](const char* bg, const char* hover, const char* pressed) {
        return QString(
            "QPushButton { background-color: %1; color: white; padding: 6px; "
            "border: 1px solid %2; border-radius: 3px; }"
            "QPushButton:hover { background-color: %2; }"
            "QPushButton:pressed { background-color: %3; }")
            .arg(bg, hover, pressed);
    };

    estop_btn_ = new QPushButton("E-STOP", this);
    estop_btn_->setCheckable(true);
    estop_btn_->setMinimumHeight(50);
    QFont estop_font = estop_btn_->font();
    estop_font.setPointSize(16);
    estop_font.setBold(true);
    estop_btn_->setFont(estop_font);
    estop_btn_->setStyleSheet(
        "QPushButton { background-color: #5a1a1a; color: white; padding: 10px; "
        "border: 2px solid #8a2a2a; border-radius: 5px; }"
        "QPushButton:hover { background-color: #6a2a2a; }"
        "QPushButton:checked { background-color: #cc0000; border-color: #ff3333; }");
    connect(estop_btn_, &QPushButton::toggled, this, &DashboardPanel::onEstopToggled);
    layout->addWidget(estop_btn_);

    auto* btn_row = new QHBoxLayout();
    btn_row->setSpacing(4);

    reset_btn_ = new QPushButton("Reset Sources", this);
    reset_btn_->setMinimumHeight(28);
    reset_btn_->setStyleSheet(btn_style("#2a4a7f", "#3a5a9f", "#1a3a6f"));
    connect(reset_btn_, &QPushButton::clicked, this,
            [this]() { emit resetSourcesRequested(); });
    btn_row->addWidget(reset_btn_);

    clear_btn_ = new QPushButton("Clear Data", this);
    clear_btn_->setMinimumHeight(28);
    clear_btn_->setStyleSheet(btn_style("#4a4a2a", "#6a6a3a", "#3a3a1a"));
    connect(clear_btn_, &QPushButton::clicked, this, &DashboardPanel::onClearAll);
    btn_row->addWidget(clear_btn_);

    settings_btn_ = new QPushButton(this);
    settings_btn_->setFixedSize(28, 28);
    settings_btn_->setToolTip("Settings");
    {
        QIcon icon = QIcon::fromTheme("preferences-system");
        if (!icon.isNull()) {
            settings_btn_->setIcon(icon);
            settings_btn_->setIconSize(QSize(16, 16));
        } else {
            settings_btn_->setText("⚙");
        }
    }
    settings_btn_->setStyleSheet(
        "QPushButton { background-color: #2d2d45; color: #ccc; padding: 2px; "
        "border: 1px solid #3a3a55; border-radius: 3px; }"
        "QPushButton:hover { background-color: #3a3a55; }");
    connect(settings_btn_, &QPushButton::clicked, this,
            [this]() { emit settingsRequested(); });
    btn_row->addWidget(settings_btn_);

    layout->addLayout(btn_row);

    auto estop_qos = rclcpp::QoS(10).reliable().transient_local();
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/estop", estop_qos);
    estop_timer_ = new QTimer(this);
    connect(estop_timer_, &QTimer::timeout, this, &DashboardPanel::publishEstopState);
    estop_timer_->start(100);
}

void DashboardPanel::setConnState(const QString& color, const QString& label)
{
    conn_indicator_->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    conn_label_->setText(label);
}

void DashboardPanel::publishSensorMask()
{
    std_msgs::msg::UInt8 msg;
    msg.data = sensor_mask_;
    sensor_mask_pub_->publish(msg);
}

void DashboardPanel::onTelemetryReceived()
{
    hb_received_ = true;
    setConnState("#33cc33", "Online");
    if (pulse_anim_) {
        pulse_anim_->stop();
        pulse_anim_->start();
    }
}

void DashboardPanel::onUptimeUpdated(float uptime_s)
{
    int secs = static_cast<int>(uptime_s);
    int mins = secs / 60;
    int hours = mins / 60;
    QString text;
    if (hours > 0)      text = QString("%1h%2m").arg(hours).arg(mins % 60);
    else if (mins > 0)  text = QString("%1m%2s").arg(mins).arg(secs % 60);
    else                text = QString("%1s").arg(secs);
    uptime_label_->setText(text);
}

void DashboardPanel::onHeartbeatCheck()
{
    if (hb_received_) {
        hb_received_ = false;
        hb_miss_count_ = 0;
        return;
    }
    if (hb_miss_count_ < 100) hb_miss_count_++;
    if (hb_miss_count_ == 1) {
        setConnState("#ccaa00", "Intermittent");
    } else if (hb_miss_count_ >= 2) {
        setConnState("#cc3333", "Offline");
        uptime_label_->setText("--");
    }
}

void DashboardPanel::onSensorToggled()
{
    sensor_mask_ &= (1 << 1);  // preserve thermal bit (driven by the video panel)
    if (mag_toggle_->isChecked()) { sensor_mask_ |= (1 << 0); mag_toggle_->setText("ON"); }
    else                          { mag_toggle_->setText("OFF"); }
    publishSensorMask();
}

void DashboardPanel::setThermalEnabled(bool enabled)
{
    if (enabled) sensor_mask_ |=  static_cast<uint8_t>(1 << 1);
    else         sensor_mask_ &= ~static_cast<uint8_t>(1 << 1);
    publishSensorMask();
}

void DashboardPanel::onEstopToggled(bool checked)
{
    estop_active_ = checked;
    publishEstopState();
}

void DashboardPanel::onAudioToggled(bool checked)
{
    if (audio_btn_) audio_btn_->setText(checked ? "ON" : "OFF");
    emit audioMonitorToggled(checked);
}

void DashboardPanel::onTranscriptionUpdated(const QString& text)
{
    transcription_->setPlainText(text);
    transcription_->moveCursor(QTextCursor::End);
}

void DashboardPanel::onMagnetometerUpdated(double x, double y, double z)
{
    mag_x_->setText(QString::number(x, 'f', 2));
    mag_y_->setText(QString::number(y, 'f', 2));
    mag_z_->setText(QString::number(z, 'f', 2));
}

void DashboardPanel::onImuUpdated(double yaw, double pitch, double roll)
{
    imu_yaw_->setText(QString::number(yaw, 'f', 1) + "°");
    imu_pitch_->setText(QString::number(pitch, 'f', 1) + "°");
    imu_roll_->setText(QString::number(roll, 'f', 1) + "°");
}

void DashboardPanel::onClearAll()
{
    for (QLabel* l : {mag_x_, mag_y_, mag_z_, imu_yaw_, imu_pitch_, imu_roll_})
        l->setText("--");
    if (speech_processor_)
        speech_processor_->clearTranscription();
}

void DashboardPanel::publishEstopState()
{
    std_msgs::msg::Bool msg;
    msg.data = estop_active_.load();
    estop_pub_->publish(msg);
}

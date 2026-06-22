#include "gui/arm_pose_panel.hpp"

#include <QAbstractItemView>
#include <QByteArray>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QPixmap>
#include <QStyle>
#include <QStyledItemDelegate>
#include <QVBoxLayout>

namespace {
constexpr char kSaveSrv[] = "/servo_node/save_pose";
constexpr char kGoSrv[] = "/servo_node/go_to_pose";
constexpr char kDeleteSrv[] = "/servo_node/delete_pose";
constexpr char kListSrv[] = "/servo_node/list_poses";
constexpr char kPlanStateTopic[] = "/servo_node/plan_state";

// Thumbnail cache resolution (saved at 2× the dropdown thumb for crispness).
constexpr int kPreviewW = 200;
constexpr int kPreviewH = 150;

QString btnStyle(const char* bg, const char* hover, const char* pressed)
{
    return QString(
        "QPushButton { background-color: %1; color: white; padding: 5px; "
        "border: 1px solid %2; border-radius: 3px; }"
        "QPushButton:hover { background-color: %2; }"
        "QPushButton:pressed { background-color: %3; }"
        "QPushButton:disabled { background-color: #2a2a35; color: #666; }")
        .arg(bg, hover, pressed);
}

// Dropdown item delegate: paints each saved pose's thumbnail next to its name so
// the preview lives in the list itself (no separate preview pane below it). The
// thumbnail pixmap is carried per-item in Qt::UserRole; the collapsed combo box
// shows just the name (text), keeping it compact.
class PoseItemDelegate : public QStyledItemDelegate {
public:
    static constexpr int kThumbW = 104;
    static constexpr int kThumbH = 58;
    static constexpr int kPad = 4;

    using QStyledItemDelegate::QStyledItemDelegate;

    QSize sizeHint(const QStyleOptionViewItem&, const QModelIndex&) const override
    {
        return QSize(kThumbW + 150, kThumbH + 2 * kPad);
    }

    void paint(QPainter* p, const QStyleOptionViewItem& opt,
               const QModelIndex& idx) const override
    {
        const bool sel = opt.state & QStyle::State_Selected;
        p->save();
        p->fillRect(opt.rect, sel ? QColor("#3a3a7a") : QColor("#1a1a2e"));

        QRect thumb(opt.rect.left() + kPad, opt.rect.top() + kPad, kThumbW, kThumbH);
        p->fillRect(thumb, QColor("#14141f"));
        QPixmap pm = idx.data(Qt::UserRole).value<QPixmap>();
        if (!pm.isNull()) {
            QPixmap s = pm.scaled(thumb.size(), Qt::KeepAspectRatio,
                                  Qt::SmoothTransformation);
            p->drawPixmap(thumb.left() + (thumb.width() - s.width()) / 2,
                          thumb.top() + (thumb.height() - s.height()) / 2, s);
        } else {
            p->setPen(QColor("#666"));
            p->drawText(thumb, Qt::AlignCenter, "no preview");
        }
        p->setPen(QColor("#333"));
        p->drawRect(thumb.adjusted(0, 0, -1, -1));

        QRect text(thumb.right() + 8, opt.rect.top(),
                   opt.rect.right() - thumb.right() - 12, opt.rect.height());
        p->setPen(sel ? QColor("#ffffff") : QColor("#c0c0c0"));
        p->drawText(text, Qt::AlignVCenter | Qt::AlignLeft,
                    idx.data(Qt::DisplayRole).toString());
        p->restore();
    }
};
}  // namespace

ArmPosePanel::ArmPosePanel(rclcpp::Node::SharedPtr node,
                           std::function<QImage()> grab_view, QWidget* parent)
    : QWidget(parent), node_(node), grab_view_(std::move(grab_view))
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(5);

    auto* hdr = new QLabel("Saved Poses", this);
    hdr->setStyleSheet("color: #aaa; font-weight: bold;");
    layout->addWidget(hdr);

    // Row: pose dropdown + Save / Go / Delete / Refresh.
    auto* row = new QHBoxLayout();
    row->setSpacing(4);

    pose_combo_ = new QComboBox(this);
    pose_combo_->setMinimumWidth(120);
    pose_combo_->setStyleSheet(
        "QComboBox { background-color: #1a1a2e; color: #c0c0c0; padding: 3px; "
        "border: 1px solid #333; border-radius: 3px; }");
    // Inline thumbnails in the dropdown rows (preview lives in the list itself).
    pose_combo_->setItemDelegate(new PoseItemDelegate(pose_combo_));
    pose_combo_->view()->setMinimumWidth(PoseItemDelegate::kThumbW + 170);
    connect(pose_combo_, &QComboBox::currentTextChanged, this, [this]() {
        QStringList names;
        for (int i = 0; i < pose_combo_->count(); ++i)
            names << pose_combo_->itemText(i);
        savePoseCache(names);
    });
    row->addWidget(pose_combo_, 1);

    save_btn_ = new QPushButton("Save", this);
    save_btn_->setToolTip("Snapshot the current end-effector pose under a name");
    save_btn_->setStyleSheet(btnStyle("#2a4a7f", "#3a5a9f", "#1a3a6f"));
    connect(save_btn_, &QPushButton::clicked, this, &ArmPosePanel::onSaveClicked);
    row->addWidget(save_btn_);

    go_btn_ = new QPushButton("Go", this);
    go_btn_->setToolTip("Plan a collision-free path and move to the selected pose");
    go_btn_->setStyleSheet(btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a"));
    connect(go_btn_, &QPushButton::clicked, this, &ArmPosePanel::onGoClicked);
    row->addWidget(go_btn_);

    delete_btn_ = new QPushButton("Del", this);
    delete_btn_->setToolTip("Delete the selected saved pose");
    delete_btn_->setStyleSheet(btnStyle("#5a2a2a", "#7a3a3a", "#3a1a1a"));
    connect(delete_btn_, &QPushButton::clicked, this, &ArmPosePanel::onDeleteClicked);
    row->addWidget(delete_btn_);

    refresh_btn_ = new QPushButton("⟳", this);
    refresh_btn_->setFixedWidth(28);
    refresh_btn_->setToolTip("Refresh the pose list from the servo");
    refresh_btn_->setStyleSheet(btnStyle("#3a3a55", "#4a4a66", "#2a2a45"));
    connect(refresh_btn_, &QPushButton::clicked, this, &ArmPosePanel::onRefreshClicked);
    row->addWidget(refresh_btn_);

    layout->addLayout(row);

    status_label_ = new QLabel("—", this);
    status_label_->setStyleSheet("color: #888; font-size: 11px;");
    layout->addWidget(status_label_);

    // ── ROS interfaces ────────────────────────────────────────────────
    save_cli_ = node_->create_client<rescue_interfaces::srv::SavePose>(kSaveSrv);
    go_cli_ = node_->create_client<rescue_interfaces::srv::GoToPose>(kGoSrv);
    delete_cli_ = node_->create_client<rescue_interfaces::srv::DeletePose>(kDeleteSrv);
    list_cli_ = node_->create_client<rescue_interfaces::srv::ListPoses>(kListSrv);

    // Latched to match the servo (transient-local): the current plan state shows
    // up immediately even if the GUI starts after the servo.
    plan_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        kPlanStateTopic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit planStateUpdated(QString::fromStdString(msg->data));
        });

    // ROS thread → Qt thread.
    connect(this, &ArmPosePanel::posesListed, this, &ArmPosePanel::onPosesListed,
            Qt::QueuedConnection);
    connect(this, &ArmPosePanel::refreshRequested, this, &ArmPosePanel::onRefreshClicked,
            Qt::QueuedConnection);
    connect(this, &ArmPosePanel::serviceResult, this, &ArmPosePanel::onServiceResult,
            Qt::QueuedConnection);
    connect(this, &ArmPosePanel::planStateUpdated, this, &ArmPosePanel::onPlanStateUpdated,
            Qt::QueuedConnection);

    QStringList cached = loadPoseCache();
    if (!cached.isEmpty()) onPosesListed(cached);
    onRefreshClicked();  // reconcile with the servo when it is available
}

void ArmPosePanel::setStatus(const QString& text, const QString& color)
{
    status_label_->setText(text);
    status_label_->setStyleSheet(QString("color: %1; font-size: 11px;").arg(color));
}

void ArmPosePanel::onSaveClicked()
{
    bool ok = false;
    QString name = QInputDialog::getText(this, "Save Pose", "Pose name:",
                                         QLineEdit::Normal, "", &ok).trimmed();
    if (!ok || name.isEmpty())
        return;
    if (!save_cli_->service_is_ready()) {
        setStatus("servo save_pose service not available", "#cc6666");
        return;
    }
    // Snapshot the twin's current view now (GUI thread); persist it only if the
    // service accepts the pose. QImage is implicitly shared, cheap to capture.
    QImage thumb = grab_view_ ? grab_view_() : QImage();

    auto req = std::make_shared<rescue_interfaces::srv::SavePose::Request>();
    req->name = name.toStdString();
    setStatus(QString("saving '%1'…").arg(name), "#4fc3f7");
    pending_select_ = name;   // select + preview this pose after the refresh
    save_cli_->async_send_request(
        req, [this, name, thumb](
                 rclcpp::Client<rescue_interfaces::srv::SavePose>::SharedFuture f) {
            auto r = f.get();
            if (r->success && !thumb.isNull()) saveThumb(name, thumb);
            emit serviceResult(QString::fromStdString(r->message), r->success);
            if (r->success) emit refreshRequested();
        });
}

void ArmPosePanel::onGoClicked()
{
    const QString name = pose_combo_->currentText().trimmed();
    if (name.isEmpty()) {
        setStatus("no pose selected", "#cc6666");
        return;
    }
    if (!go_cli_->service_is_ready()) {
        setStatus("servo go_to_pose service not available", "#cc6666");
        return;
    }
    auto req = std::make_shared<rescue_interfaces::srv::GoToPose::Request>();
    req->name = name.toStdString();
    req->use_pose = false;
    setStatus(QString("planning to '%1'…").arg(name), "#4fc3f7");
    go_cli_->async_send_request(
        req, [this](rclcpp::Client<rescue_interfaces::srv::GoToPose>::SharedFuture f) {
            auto r = f.get();
            emit serviceResult(QString::fromStdString(r->message), r->success);
        });
}

void ArmPosePanel::onDeleteClicked()
{
    const QString name = pose_combo_->currentText().trimmed();
    if (name.isEmpty()) {
        setStatus("no pose selected", "#cc6666");
        return;
    }
    if (!delete_cli_->service_is_ready()) {
        setStatus("servo delete_pose service not available", "#cc6666");
        return;
    }
    auto req = std::make_shared<rescue_interfaces::srv::DeletePose::Request>();
    req->name = name.toStdString();
    delete_cli_->async_send_request(
        req, [this, name](
                 rclcpp::Client<rescue_interfaces::srv::DeletePose>::SharedFuture f) {
            auto r = f.get();
            if (r->success) removeThumb(name);
            emit serviceResult(QString::fromStdString(r->message), r->success);
            if (r->success) emit refreshRequested();
        });
}

void ArmPosePanel::onRefreshClicked()
{
    if (!list_cli_->service_is_ready()) {
        setStatus(pose_combo_->count() > 0 ? "servo not connected; showing saved cache"
                                           : "servo not connected",
                  "#ccaa00");
        return;
    }
    list_cli_->async_send_request(
        std::make_shared<rescue_interfaces::srv::ListPoses::Request>(),
        [this](rclcpp::Client<rescue_interfaces::srv::ListPoses>::SharedFuture f) {
            auto r = f.get();
            QStringList names;
            for (const auto& n : r->names) names << QString::fromStdString(n);
            emit posesListed(names);
        });
}

void ArmPosePanel::onPosesListed(const QStringList& names)
{
    // Prefer a just-saved pose; otherwise keep the current selection.
    const QString want = !pending_select_.isEmpty() ? pending_select_
                                                     : pose_combo_->currentText();
    pending_select_.clear();
    pose_combo_->blockSignals(true);
    pose_combo_->clear();
    for (const QString& n : names) {
        pose_combo_->addItem(n);
        // Carry the pose's saved render as the item's thumbnail (drawn inline by
        // PoseItemDelegate). Missing snapshot → the delegate shows "no preview".
        QPixmap pm(thumbPath(n));
        if (!pm.isNull())
            pose_combo_->setItemData(pose_combo_->count() - 1, pm, Qt::UserRole);
    }
    int idx = pose_combo_->findText(want);
    pose_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
    pose_combo_->blockSignals(false);
    savePoseCache(names);
}

QString ArmPosePanel::savedPosesPath() const
{
    return QDir::homePath() + "/.config/robocorea_gui/saved_poses.json";
}

QStringList ArmPosePanel::loadPoseCache()
{
    QFile f(savedPosesPath());
    if (!f.open(QIODevice::ReadOnly))
        return {};

    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject())
        return {};

    QStringList names;
    const QJsonArray arr = doc.object().value("poses").toArray();
    for (const QJsonValue& v : arr) {
        QString name = v.toString().trimmed();
        if (!name.isEmpty() && !names.contains(name))
            names << name;
    }
    names.sort(Qt::CaseInsensitive);
    pending_select_ = doc.object().value("selected").toString();
    return names;
}

void ArmPosePanel::savePoseCache(const QStringList& names) const
{
    QStringList cleaned;
    for (const QString& n : names) {
        QString name = n.trimmed();
        if (!name.isEmpty() && !cleaned.contains(name))
            cleaned << name;
    }
    cleaned.sort(Qt::CaseInsensitive);

    QJsonArray arr;
    for (const QString& name : cleaned)
        arr.append(name);

    QJsonObject root;
    root["version"] = 1;
    root["poses"] = arr;
    root["selected"] = pose_combo_->currentText();

    const QString path = savedPosesPath();
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

QString ArmPosePanel::thumbPath(const QString& name) const
{
    // Filename-safe, collision-free key from the pose name.
    const QString key = QString::fromLatin1(
        name.toUtf8().toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals));
    return QDir::homePath() + "/.config/robocorea_gui/pose_thumbs/" + key + ".png";
}

void ArmPosePanel::saveThumb(const QString& name, const QImage& img) const
{
    const QString path = thumbPath(name);
    QDir().mkpath(QFileInfo(path).absolutePath());
    img.scaled(kPreviewW * 2, kPreviewH * 2, Qt::KeepAspectRatio,
               Qt::SmoothTransformation)
        .save(path, "PNG");
}

void ArmPosePanel::removeThumb(const QString& name) const
{
    QFile::remove(thumbPath(name));
}

void ArmPosePanel::onServiceResult(const QString& text, bool ok)
{
    setStatus(text, ok ? "#8afa8a" : "#cc6666");
}

void ArmPosePanel::onPlanStateUpdated(const QString& text)
{
    // Live progress from the servo (planning / moving N/M / reached / aborted).
    QString color = "#4fc3f7";
    if (text.startsWith("reached")) color = "#8afa8a";
    else if (text.startsWith("aborted") || text.contains("fail") ||
             text == "unreachable") color = "#cc6666";
    setStatus(text, color);
}

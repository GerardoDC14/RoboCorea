#pragma once

#include <QComboBox>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QWidget>

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rescue_interfaces/srv/save_pose.hpp>
#include <rescue_interfaces/srv/go_to_pose.hpp>
#include <rescue_interfaces/srv/delete_pose.hpp>
#include <rescue_interfaces/srv/list_poses.hpp>

// Saved-pose controls for the digital-twin section. A thin client of the
// servo_node pose services (save/go/delete/list): "Save" snapshots the arm's
// current end-effector pose under a name; "Go" re-solves IK + RRT-plans a
// collision-free path back to it (executed server-side, the twin follows
// /joint_states). The status line mirrors the servo's /servo_node/plan_state.
//
// Each pose's saved render is shown as a thumbnail **inline in the dropdown
// list** (one per entry, via a custom item delegate) rather than a separate
// preview pane, so the operator can eyeball poses while picking from the list.
//
// Service results arrive on the GUI's ROS spin thread and are marshaled to the
// Qt thread via queued signals (the established *Updated pattern).
class ArmPosePanel : public QWidget {
    Q_OBJECT
public:
    // `grab_view` returns a snapshot of the digital-twin render (used to make the
    // saved-pose preview thumbnail). May be empty; then previews are skipped.
    explicit ArmPosePanel(rclcpp::Node::SharedPtr node,
                          std::function<QImage()> grab_view,
                          QWidget* parent = nullptr);

signals:
    void posesListed(const QStringList& names);
    void refreshRequested();
    void serviceResult(const QString& text, bool ok);
    void planStateUpdated(const QString& text);

private slots:
    void onSaveClicked();
    void onGoClicked();
    void onDeleteClicked();
    void onRefreshClicked();
    void onPosesListed(const QStringList& names);
    void onServiceResult(const QString& text, bool ok);
    void onPlanStateUpdated(const QString& text);

private:
    void setStatus(const QString& text, const QString& color = "#888");
    QString savedPosesPath() const;
    QStringList loadPoseCache();
    void savePoseCache(const QStringList& names) const;
    QString thumbPath(const QString& name) const;
    void saveThumb(const QString& name, const QImage& img) const;
    void removeThumb(const QString& name) const;

    rclcpp::Node::SharedPtr node_;
    std::function<QImage()> grab_view_;
    QString pending_select_;   // pose to select after the next list refresh

    QComboBox* pose_combo_;
    QPushButton* save_btn_;
    QPushButton* go_btn_;
    QPushButton* delete_btn_;
    QPushButton* refresh_btn_;
    QLabel* status_label_;

    rclcpp::Client<rescue_interfaces::srv::SavePose>::SharedPtr save_cli_;
    rclcpp::Client<rescue_interfaces::srv::GoToPose>::SharedPtr go_cli_;
    rclcpp::Client<rescue_interfaces::srv::DeletePose>::SharedPtr delete_cli_;
    rclcpp::Client<rescue_interfaces::srv::ListPoses>::SharedPtr list_cli_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_state_sub_;
};

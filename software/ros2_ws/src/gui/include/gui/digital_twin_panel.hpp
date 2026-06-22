#pragma once

#include <QWidget>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>

#include "gui/urdf_viewer.hpp"
#include "gui/arm_pose_panel.hpp"

class DigitalTwinPanel : public QWidget {
    Q_OBJECT
public:
    explicit DigitalTwinPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(2);
        viewer_ = new UrdfViewer(node, this);
        layout->addWidget(viewer_, 1);              // 3-D view takes the slack
        // Saved-pose controls below it; the grab callback feeds pose previews.
        pose_panel_ = new ArmPosePanel(
            node, [v = viewer_]() { return v->grabFramebuffer(); }, this);
        layout->addWidget(pose_panel_);
    }

private:
    UrdfViewer* viewer_;
    ArmPosePanel* pose_panel_;
};

#pragma once

#include <QWidget>

#include <rclcpp/rclcpp.hpp>

class UrdfViewer;

// Standalone top-level window showing the live SLAM map as a textured 3-D floor
// (from /map) with the full robot URDF placed at its map->base_footprint pose.
// It reuses UrdfViewer in "map mode"; SLAM/EKF run on the robot and this only
// visualises /map + TF + /joint_states over DDS. Mouse: drag = orbit, middle-
// drag = pan, wheel = zoom.
class MapWindow : public QWidget {
    Q_OBJECT
public:
    explicit MapWindow(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

private:
    UrdfViewer* viewer_{nullptr};
};

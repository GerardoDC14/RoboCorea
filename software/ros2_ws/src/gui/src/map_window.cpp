#include "gui/map_window.hpp"
#include "gui/urdf_viewer.hpp"

#include <QVBoxLayout>

MapWindow::MapWindow(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent)
{
    setWindowTitle("RoboCorea — Map");
    resize(900, 800);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // Reuse the URDF renderer in map mode: textured /map floor + robot at its
    // map->base_footprint pose, posed from /joint_states like the digital twin.
    viewer_ = new UrdfViewer(node, this);
    viewer_->setMapMode(true);
    layout->addWidget(viewer_);
}

#pragma once

#include <QMainWindow>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

class CameraHub;
class VideoPanel;
class SourceManager;
class DashboardPanel;
class OdometryPanel;
class DigitalTwinPanel;
class GstAvStream;
class QWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onSourcesUpdated();

private:
    void startRosSpinThread();
    QWidget* buildRightColumn();      // digital twin (top) + dashboard (bottom)
    QWidget* wrapScroll(QWidget* w);  // scrollable container for a tall panel
#ifdef HAVE_GSTREAMER
    void setupAvStreams();   // create + register the C920 A/V SRT receivers
#endif

    rclcpp::Node::SharedPtr node_;

    VideoPanel* video_panel_;
    SourceManager* source_manager_;
    DashboardPanel* dashboard_panel_;
    OdometryPanel* odometry_panel_;
    DigitalTwinPanel* digital_twin_panel_;
    std::shared_ptr<CameraHub> camera_hub_;
#ifdef HAVE_GSTREAMER
    std::vector<std::shared_ptr<GstAvStream>> av_streams_;
#endif

    std::thread ros_thread_;
    std::atomic<bool> ros_running_{true};
};

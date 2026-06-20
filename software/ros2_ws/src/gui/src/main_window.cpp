#include "gui/main_window.hpp"
#include "gui/camera_hub.hpp"
#include "gui/video_panel.hpp"
#include "gui/source_manager.hpp"
#include "gui/dashboard_panel.hpp"
#include "gui/odometry_panel.hpp"
#include "gui/digital_twin_panel.hpp"
#include "gui/filter_registry.hpp"
#include "gui/app_settings.hpp"
#ifdef HAVE_GSTREAMER
#include "gui/speech_processor.hpp"
#include "gui/gst_av_stream.hpp"
#endif

#include <QFrame>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>

#include <memory>

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QMainWindow(parent), node_(node)
{
    setWindowTitle("RoboCorea Operator Console");
    resize(1600, 900);

    AppSettings::instance().load();

    camera_hub_ = std::make_shared<CameraHub>();
    video_panel_ = new VideoPanel(node_, camera_hub_, this);

    // Layout: video on the left, and a right section with the digital twin on
    // top over the odometry + dashboard panels side by side below.
    auto* main_splitter = new QSplitter(Qt::Horizontal, this);
    main_splitter->addWidget(video_panel_);
    main_splitter->addWidget(buildRightColumn());
    main_splitter->setStretchFactor(0, 1);  // video takes the slack on resize
    main_splitter->setSizes({980, 620});
    setCentralWidget(main_splitter);

    // Populate the per-widget filter dropdowns from the CV filter registry
    // (registerFilters() runs in main() before the window is built).
    QStringList filter_names{"None"};
    for (const auto& name : FilterRegistry::instance().getFilterNames())
        filter_names << QString::fromStdString(name);
    video_panel_->updateFilters(filter_names);

    // A/V SRT receivers must be registered with CameraHub before sources are
    // discovered (so the "av:<i>" ids resolve to live frames).
#ifdef HAVE_GSTREAMER
    setupAvStreams();
#endif

    source_manager_ = new SourceManager(node_, camera_hub_, this);
    connect(source_manager_, &SourceManager::sourcesUpdated,
            this, &MainWindow::onSourcesUpdated);

    // Selecting a thermal source anywhere enables the thermal sensor bit.
    connect(video_panel_, &VideoPanel::thermalActiveChanged,
            dashboard_panel_, &DashboardPanel::setThermalEnabled);
    connect(dashboard_panel_, &DashboardPanel::resetSourcesRequested,
            source_manager_, &SourceManager::discoverSources);
    connect(dashboard_panel_, &DashboardPanel::settingsRequested, this, [this]() {
        statusBar()->showMessage("Settings dialog not implemented yet", 3000);
    });
#ifdef HAVE_GSTREAMER
    // Audio-monitor toggle mutes/unmutes the speaker side of every A/V stream.
    connect(dashboard_panel_, &DashboardPanel::audioMonitorToggled, this,
            [this](bool en) {
                for (auto& s : av_streams_) s->setPlaybackEnabled(en);
            });
#endif

    startRosSpinThread();
    source_manager_->discoverSources();
}

MainWindow::~MainWindow()
{
    ros_running_ = false;
    if (ros_thread_.joinable())
        ros_thread_.join();
#ifdef HAVE_GSTREAMER
    // Stop A/V receivers before the dashboard/speech processor (QObject children)
    // are torn down, so no audio callback fires into a dead SpeechProcessor.
    av_streams_.clear();
#endif
}

#ifdef HAVE_GSTREAMER
void MainWindow::setupAvStreams()
{
    auto streams = AppSettings::instance().videoStreams();
    std::string default_host;
    bool playback;
    {
        std::lock_guard<std::mutex> lk(AppSettings::instance().video_mutex);
        default_host = AppSettings::instance().default_robot_host;
    }
    playback = AppSettings::instance().audio_start_enabled.load();

    SpeechProcessor* speech = dashboard_panel_->speechProcessor();

    for (size_t i = 0; i < streams.size(); ++i) {
        const auto& s = streams[i];
        if (!s.audio) continue;
        const std::string host = s.host.empty() ? default_host : s.host;
        if (host.empty()) continue;

        auto av = std::make_shared<GstAvStream>(
            host, s.port, s.latency_ms,
            [speech](const int16_t* pcm, size_t n) {
                if (speech) speech->pushAudio(pcm, n);
            });
        av->setPlaybackEnabled(playback);
        av->start();

        const std::string id = "av:" + std::to_string(i);
        std::weak_ptr<GstAvStream> w = av;
        camera_hub_->registerFrameSource(
            id,
            [w]() { auto sp = w.lock(); return sp ? sp->latestVideoFrame() : cv::Mat(); },
            [w]() { auto sp = w.lock(); return sp && sp->connected(); });

        av_streams_.push_back(std::move(av));
    }
}
#endif  // HAVE_GSTREAMER

QWidget* MainWindow::wrapScroll(QWidget* w)
{
    auto* sa = new QScrollArea(this);
    sa->setWidgetResizable(true);
    sa->setWidget(w);
    sa->setFrameShape(QFrame::NoFrame);
    return sa;
}

QWidget* MainWindow::buildRightColumn()
{
    // Top: the digital twin (3-D URDF view from /robot_description + /joint_states),
    // spanning the full width of the right section.
    digital_twin_panel_ = new DigitalTwinPanel(node_, this);

    // Bottom: odometry (telemetry) and dashboard side by side, each scrollable.
    odometry_panel_ = new OdometryPanel(node_, this);
    dashboard_panel_ = new DashboardPanel(node_, this);

    auto* bottom_splitter = new QSplitter(Qt::Horizontal, this);
    bottom_splitter->addWidget(wrapScroll(odometry_panel_));
    bottom_splitter->addWidget(wrapScroll(dashboard_panel_));
    bottom_splitter->setSizes({320, 320});

    auto* right_splitter = new QSplitter(Qt::Vertical, this);
    right_splitter->addWidget(digital_twin_panel_);
    right_splitter->addWidget(bottom_splitter);
    right_splitter->setSizes({420, 480});
    return right_splitter;
}

void MainWindow::startRosSpinThread()
{
    ros_thread_ = std::thread([this]() {
        while (ros_running_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void MainWindow::onSourcesUpdated()
{
    video_panel_->updateSources(
        source_manager_->sourceNames(),
        source_manager_->sourceIdentifiers());
}

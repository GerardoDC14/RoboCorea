#pragma once

#include <QGridLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <vector>

class CameraHub;
class VideoWidget;

// A 2×2 grid of VideoWidgets. Clicking a cell enlarges it to span the whole
// grid; clicking again restores. The odometry/dashboard panels live in the right
// section of the MainWindow splitter, not in this grid.
class VideoPanel : public QWidget {
    Q_OBJECT
public:
    explicit VideoPanel(rclcpp::Node::SharedPtr node,
                        std::shared_ptr<CameraHub> hub,
                        QWidget* parent = nullptr);

    void updateSources(const QStringList& names, const QStringList& identifiers);
    void updateFilters(const QStringList& names);

signals:
    void thermalActiveChanged(bool active);

private slots:
    void onWidgetClicked(int index);
    void onWidgetThermalChanged(bool active);

private:
    static constexpr int ROWS = 2;
    static constexpr int COLS = 2;
    static constexpr int CELLS = ROWS * COLS;

    QGridLayout* grid_;
    std::vector<VideoWidget*> widgets_;
    int enlarged_index_{-1};
    std::atomic<int> thermal_count_{0};
};

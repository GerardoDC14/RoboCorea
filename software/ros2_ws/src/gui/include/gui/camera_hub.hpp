#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Owns shared cv::VideoCapture instances so multiple VideoWidgets can view the
// same pull-based source simultaneously with a single decode.
//
// Sources are identified by a URI-style string:
//   "local:N"        → V4L2 device /dev/videoN (RF driving cams, USB webcams)
//   "gst:<pipeline>" → an arbitrary GStreamer pipeline ending in `appsink`
//                      (the C920 SRT streams from the Jetson live here)
//
// Each distinct source gets one capture thread; consumers call getLatestFrame()
// to obtain a clone of the most recent frame. Reference-counted: the thread
// starts on the first subscribe() and stops when the last consumer
// unsubscribe()s. The capture thread keeps trying to (re)open its source, so a
// stream that is not up yet — or that drops mid-run (e.g. the SRT link) —
// recovers automatically without tearing down the subscription.
class CameraHub {
public:
    CameraHub() = default;
    ~CameraHub();

    CameraHub(const CameraHub&) = delete;
    CameraHub& operator=(const CameraHub&) = delete;

    void subscribe(const std::string& source_id);
    void unsubscribe(const std::string& source_id);
    cv::Mat getLatestFrame(const std::string& source_id);
    bool isConnected(const std::string& source_id) const;
    std::vector<std::string> activeSourceIds() const;

    // Register a source whose frames come from an external producer (e.g. the
    // native A/V GStreamer receiver) rather than a hub-owned capture thread.
    // subscribe()/unsubscribe() on such ids are no-ops; getLatestFrame()/
    // isConnected() defer to the supplied getters.
    void registerFrameSource(const std::string& source_id,
                             std::function<cv::Mat()> frame_getter,
                             std::function<bool()> connected_getter);
    void unregisterFrameSource(const std::string& source_id);

private:
    struct Stream {
        cv::VideoCapture capture;
        std::thread thread;
        std::atomic<bool> running{true};
        std::atomic<bool> connected{false};
        std::mutex frame_mutex;
        cv::Mat latest_frame;
        int ref_count{0};
    };

    struct ExternalSource {
        std::function<cv::Mat()> frame_getter;
        std::function<bool()> connected_getter;
    };

    void captureLoop(const std::string& source_id, Stream* stream);
    static bool openSource(cv::VideoCapture& cap, const std::string& source_id);

    mutable std::mutex streams_mutex_;
    std::map<std::string, std::unique_ptr<Stream>> streams_;
    std::map<std::string, ExternalSource> external_;
};

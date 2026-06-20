#pragma once

#include <gst/gst.h>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// Native-GStreamer receiver for ONE SRT stream that carries H.264 video AND Opus
// audio in a single MPEG-TS (the C920 A/V stream — onboard H.264 + its mic).
//
// A single SRT listener serves one connection, so the muxed stream must be
// demuxed in one pipeline; OpenCV's VideoCapture can only pull one video sink,
// hence this native path:
//
//   srtsrc(caller) ! tsdemux ─┬─ queue ! h264parse ! avdec_h264 ! videoconvert ! BGR ! appsink(video)
//                             └─ queue ! opusparse ! opusdec plc=true ! audioconvert ! tee ─┬─ volume ! autoaudiosink
//                                                                                           └─ S16LE 16k mono ! appsink(audio→Vosk)
//
// Video frames are exposed as cv::Mat (consumed via CameraHub like any source);
// decoded PCM is pushed to an audio callback (Vosk); playback is on by default
// and mutable via setPlaybackEnabled(). Reconnects automatically on error/EOS.
class GstAvStream {
public:
    using AudioCallback = std::function<void(const int16_t* pcm, size_t samples)>;

    GstAvStream(const std::string& host, int port, int latency_ms,
                AudioCallback audio_cb);
    ~GstAvStream();

    GstAvStream(const GstAvStream&) = delete;
    GstAvStream& operator=(const GstAvStream&) = delete;

    void start();
    void stop();

    cv::Mat latestVideoFrame();   // clone of the most recent decoded frame
    bool connected() const;       // a frame arrived within the last ~2 s
    void setPlaybackEnabled(bool enabled);  // mute/unmute the speaker monitor

private:
    std::string buildPipeline() const;
    bool launch();
    void teardown();
    void monitorLoop();           // watches the bus, restarts on error/EOS

    static GstFlowReturn onVideoSample(GstElement* sink, gpointer self);
    static GstFlowReturn onAudioSample(GstElement* sink, gpointer self);

    const std::string host_;
    const int port_;
    const int latency_ms_;
    AudioCallback audio_cb_;

    GstElement* pipeline_{nullptr};
    GstElement* monitor_vol_{nullptr};
    std::mutex vol_mutex_;        // guards monitor_vol_ (GUI vs reconnect thread)

    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    std::atomic<std::int64_t> last_frame_ms_{0};

    std::atomic<bool> running_{false};
    std::atomic<bool> playback_enabled_{true};
    std::thread monitor_thread_;
};

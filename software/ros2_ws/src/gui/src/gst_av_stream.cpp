#include "gui/gst_av_stream.hpp"

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

namespace {
std::int64_t now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

GstAvStream::GstAvStream(const std::string& host, int port, int latency_ms,
                         AudioCallback audio_cb)
    : host_(host), port_(port), latency_ms_(latency_ms),
      audio_cb_(std::move(audio_cb))
{
}

GstAvStream::~GstAvStream()
{
    stop();
}

std::string GstAvStream::buildPipeline() const
{
    // Audio appsink is S16LE 16 kHz mono — exactly what Vosk wants. opusdec
    // plc=true conceals lost packets; the `volume` element gates playback.
    return
        "srtsrc uri=\"srt://" + host_ + ":" + std::to_string(port_) +
        "?mode=caller&latency=" + std::to_string(latency_ms_) + "\" ! "
        "tsdemux name=d "
        "d. ! queue ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=vsink emit-signals=true sync=false max-buffers=1 drop=true "
        "d. ! queue ! opusparse ! opusdec plc=true ! audioconvert ! audioresample ! "
        "tee name=t "
        "t. ! queue ! volume name=monitor_vol ! autoaudiosink sync=false "
        "t. ! queue ! audioconvert ! audioresample ! "
        "audio/x-raw,format=S16LE,rate=16000,channels=1 ! "
        "appsink name=asink emit-signals=true sync=false max-buffers=20 drop=true";
}

bool GstAvStream::launch()
{
    GError* err = nullptr;
    pipeline_ = gst_parse_launch(buildPipeline().c_str(), &err);
    if (!pipeline_) {
        if (err) g_error_free(err);
        return false;
    }
    if (err) g_error_free(err);  // non-fatal warnings

    GstElement* vsink = gst_bin_get_by_name(GST_BIN(pipeline_), "vsink");
    GstElement* asink = gst_bin_get_by_name(GST_BIN(pipeline_), "asink");
    GstElement* vol = gst_bin_get_by_name(GST_BIN(pipeline_), "monitor_vol");

    if (vsink) {
        g_signal_connect(vsink, "new-sample", G_CALLBACK(&GstAvStream::onVideoSample), this);
        gst_object_unref(vsink);
    }
    if (asink) {
        g_signal_connect(asink, "new-sample", G_CALLBACK(&GstAvStream::onAudioSample), this);
        gst_object_unref(asink);
    }
    {
        std::lock_guard<std::mutex> lock(vol_mutex_);
        monitor_vol_ = vol;
        if (monitor_vol_)
            g_object_set(monitor_vol_, "volume", playback_enabled_.load() ? 1.0 : 0.0, nullptr);
    }

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    return true;
}

void GstAvStream::teardown()
{
    {
        std::lock_guard<std::mutex> lock(vol_mutex_);
        if (monitor_vol_) {
            gst_object_unref(monitor_vol_);
            monitor_vol_ = nullptr;
        }
    }
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

void GstAvStream::start()
{
    if (running_.exchange(true))
        return;
    monitor_thread_ = std::thread(&GstAvStream::monitorLoop, this);
}

void GstAvStream::stop()
{
    if (!running_.exchange(false))
        return;
    if (monitor_thread_.joinable())
        monitor_thread_.join();
}

void GstAvStream::monitorLoop()
{
    while (running_) {
        if (!pipeline_ && !launch()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        GstBus* bus = gst_element_get_bus(pipeline_);
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 200 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        gst_object_unref(bus);

        if (msg) {
            // Error or end-of-stream → tear down and let the loop relaunch
            // (robot restarted its streamer, link dropped, etc.).
            gst_message_unref(msg);
            teardown();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    teardown();
}

GstFlowReturn GstAvStream::onVideoSample(GstElement* sink, gpointer self)
{
    auto* me = static_cast<GstAvStream*>(self);
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample)
        return GST_FLOW_OK;

    GstCaps* caps = gst_sample_get_caps(sample);
    GstVideoInfo info;
    if (caps && gst_video_info_from_caps(&info, caps)) {
        GstBuffer* buf = gst_sample_get_buffer(sample);
        GstVideoFrame frame;
        if (buf && gst_video_frame_map(&frame, &info, buf, GST_MAP_READ)) {
            int w = GST_VIDEO_FRAME_WIDTH(&frame);
            int h = GST_VIDEO_FRAME_HEIGHT(&frame);
            int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
            cv::Mat view(h, w, CV_8UC3,
                         GST_VIDEO_FRAME_PLANE_DATA(&frame, 0), stride);
            {
                std::lock_guard<std::mutex> lock(me->frame_mutex_);
                me->latest_frame_ = view.clone();
            }
            me->last_frame_ms_ = now_ms();
            gst_video_frame_unmap(&frame);
        }
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

GstFlowReturn GstAvStream::onAudioSample(GstElement* sink, gpointer self)
{
    auto* me = static_cast<GstAvStream*>(self);
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample)
        return GST_FLOW_OK;

    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (buf && me->audio_cb_ && gst_buffer_map(buf, &map, GST_MAP_READ)) {
        me->audio_cb_(reinterpret_cast<const int16_t*>(map.data),
                      map.size / sizeof(int16_t));
        gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

cv::Mat GstAvStream::latestVideoFrame()
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_frame_.empty() ? cv::Mat() : latest_frame_.clone();
}

bool GstAvStream::connected() const
{
    return now_ms() - last_frame_ms_.load() < 2000;
}

void GstAvStream::setPlaybackEnabled(bool enabled)
{
    playback_enabled_ = enabled;
    std::lock_guard<std::mutex> lock(vol_mutex_);
    if (monitor_vol_)
        g_object_set(monitor_vol_, "volume", enabled ? 1.0 : 0.0, nullptr);
}

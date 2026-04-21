#ifndef ASTRACAMERA_H
#define ASTRACAMERA_H

#include <QObject>
#include <openni.h>
#include <opencv2/opencv.hpp>
#include <string>

class AstraCamera : public QObject
{
    Q_OBJECT
public:
    explicit AstraCamera(QObject* parent = nullptr);
    ~AstraCamera();

    bool initialize();
    void shutdown();
    bool startStreams();
    void stopStreams();
    bool readFrame(cv::Mat& colorMat, cv::Mat& depthMat);

    bool getIntrinsics(float& fx, float& fy, float& cx, float& cy) const;

    bool isInitialized() const { return m_initialized; }
    bool isEmulationActive() const { return m_emulation; }
    std::string getLastError() const { return m_lastError; }

    void setEmulationMode(bool enable) { m_emulation = enable; }

private:
    openni::Device m_device;
    openni::VideoStream m_depthStream;
    openni::VideoFrameRef m_depthFrame;

    cv::VideoCapture m_colorCapture;
    int m_colorIndex = -1;

    bool m_initialized = false;
    bool m_emulation = false;
    std::string m_lastError;

    float m_fx = 570.0f, m_fy = 570.0f, m_cx = 320.0f, m_cy = 240.0f;

    bool openColorCamera();
    void enableEmulation(const std::string &reason);
    void generateTestFrames(cv::Mat& color, cv::Mat& depth);
};

#endif // ASTRACAMERA_H
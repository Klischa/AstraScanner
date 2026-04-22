#ifndef ASTRACAMERA_H
#define ASTRACAMERA_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <string>
#include "IDepthSensor.h"

// OpenNI2 SDK доступен только там, где его положил пользователь (обычно
// только Windows с Orbbec-драйверами). На остальных платформах собираем
// без OpenNI2; AstraCamera автоматически переключается в emulation-mode и
// отдаёт синтетические кадры, чтобы приложение хотя бы запускалось.
#ifdef ASTRA_HAVE_OPENNI2
#include <openni.h>
#endif

// Реализация IDepthSensor для камеры Orbbec Astra Pro.
// Наследует QObject (для Qt-сигналов) и IDepthSensor (для полиморфизма).
class AstraCamera : public QObject, public IDepthSensor
{
    Q_OBJECT
public:
    explicit AstraCamera(QObject* parent = nullptr);
    ~AstraCamera() override;

    // --- IDepthSensor ---
    bool initialize() override;
    void shutdown() override;
    bool startStreams() override;
    void stopStreams() override;
    bool readFrame(cv::Mat& colorMat, cv::Mat& depthMat) override;
    bool getIntrinsics(float& fx, float& fy, float& cx, float& cy) const override;
    bool isInitialized() const override { return m_initialized; }
    bool isEmulationActive() const override { return m_emulation; }
    std::string getLastError() const override { return m_lastError; }
    std::string sensorName() const override { return "Orbbec Astra Pro (OpenNI2)"; }

    void setEmulationMode(bool enable) { m_emulation = enable; }

private:
#ifdef ASTRA_HAVE_OPENNI2
    openni::Device m_device;
    openni::VideoStream m_depthStream;
    openni::VideoFrameRef m_depthFrame;
#endif

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

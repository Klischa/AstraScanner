#include "AstraCamera.h"
#include <QThread>
#include <QDebug>
#include <cstdlib>

AstraCamera::AstraCamera(QObject *parent) : QObject(parent) {}

AstraCamera::~AstraCamera()
{
    shutdown();
}

bool AstraCamera::openColorCamera()
{
    // Список предпочитаемых бэкендов. Приоритет MSMF отключается в main() до
    // первой инициализации OpenCV VideoIO (см. main.cpp).

    std::vector<cv::VideoCaptureAPIs> backends = {
        cv::CAP_DSHOW,   // DirectShow (наиболее стабилен для UVC)
        cv::CAP_ANY,     // Любой другой
        cv::CAP_MSMF     // Media Foundation (в крайнем случае)
    };

    for (int index = 0; index < 10; ++index) {
        for (auto backend : backends) {
            cv::VideoCapture capture(index, backend);
            if (!capture.isOpened()) continue;

            // Пробуем установить параметры
            capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
            capture.set(cv::CAP_PROP_FPS, 30);
            capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

            // Проверяем, что камера действительно отдаёт кадры
            cv::Mat frame;
            if (capture.read(frame) && !frame.empty()) {
                m_colorCapture = std::move(capture);
                m_colorIndex = index;
                qInfo() << "Opened UVC camera index" << index 
                        << "with backend" << backend;
                return true;
            }
        }
    }

    qCritical() << "Could not open UVC camera with any backend";
    return false;
}

void AstraCamera::enableEmulation(const std::string &reason)
{
    qWarning() << "Switching to emulation mode:" << QString::fromStdString(reason);
    m_emulation = true;
    m_initialized = true;
}

bool AstraCamera::initialize()
{
    if (m_emulation) {
        qInfo() << "Emulation mode enabled";
        m_initialized = true;
        return true;
    }

    qDebug() << "Initializing OpenNI...";
    openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK) {
        m_lastError = "OpenNI init failed: " + std::string(openni::OpenNI::getExtendedError());
        qCritical() << m_lastError.c_str();
        enableEmulation(m_lastError);
        return true;
    }

    rc = m_device.open(openni::ANY_DEVICE);
    if (rc != openni::STATUS_OK) {
        m_lastError = "Failed to open OpenNI device: " + std::string(openni::OpenNI::getExtendedError());
        qCritical() << m_lastError.c_str();
        openni::OpenNI::shutdown();
        enableEmulation(m_lastError);
        return true;
    }

    struct OBCameraParams {
        float l_intr_p[4];
        float r_intr_p[4];
        float r2l_r[9];
        float r2l_t[3];
        float k[5];
        int is_mirror;
    };
    OBCameraParams params{};
    int dataSize = sizeof(params);
    rc = m_device.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&params, &dataSize);
    if (rc == openni::STATUS_OK) {
        m_fx = params.l_intr_p[0];
        m_fy = params.l_intr_p[1];
        m_cx = params.l_intr_p[2];
        m_cy = params.l_intr_p[3];
        qInfo() << "Intrinsics from device: fx=" << m_fx << "fy=" << m_fy << "cx=" << m_cx << "cy=" << m_cy;
    } else {
        qWarning() << "Failed to get camera intrinsics";
    }

    if (!openColorCamera()) {
        m_lastError = "Unable to open UVC RGB camera";
        qCritical() << m_lastError.c_str();
        m_device.close();
        openni::OpenNI::shutdown();
        enableEmulation(m_lastError);
        return true;
    }

    if (!m_device.hasSensor(openni::SENSOR_DEPTH)) {
        m_lastError = "Depth sensor not available";
        qCritical() << m_lastError.c_str();
        m_device.close();
        openni::OpenNI::shutdown();
        m_colorCapture.release();
        enableEmulation(m_lastError);
        return true;
    }

    m_initialized = true;
    return true;
}

void AstraCamera::shutdown()
{
    stopStreams();
    if (m_device.isValid()) m_device.close();
    if (!m_emulation) openni::OpenNI::shutdown();
    if (m_colorCapture.isOpened()) m_colorCapture.release();
    m_initialized = false;
}

bool AstraCamera::startStreams()
{
    if (m_emulation) return true;
    if (!m_initialized) return false;

    if (!m_device.hasSensor(openni::SENSOR_DEPTH)) {
        m_lastError = "Device has no depth sensor";
        return false;
    }

    openni::Status rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK) {
        m_lastError = "Depth stream create failed";
        return false;
    }

    const openni::Array<openni::VideoMode>& modes = m_depthStream.getSensorInfo().getSupportedVideoModes();
    for (int i = 0; i < modes.getSize(); ++i) {
        openni::VideoMode mode = modes[i];
        if (mode.getResolutionX() == 640 && mode.getResolutionY() == 480 &&
            mode.getFps() == 30 && mode.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM) {
            m_depthStream.setVideoMode(mode);
            break;
        }
    }

    rc = m_depthStream.start();
    if (rc != openni::STATUS_OK) {
        m_lastError = "Depth stream start failed";
        return false;
    }

    if (m_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
        m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }

    return true;
}

void AstraCamera::stopStreams()
{
    if (!m_emulation && m_depthStream.isValid()) {
        m_depthStream.stop();
        m_depthStream.destroy();
    }
}

bool AstraCamera::readFrame(cv::Mat &colorMat, cv::Mat &depthMat)
{
    if (m_emulation) {
        generateTestFrames(colorMat, depthMat);
        return true;
    }

    cv::Mat tempColor;
    if (!m_colorCapture.read(tempColor) || tempColor.empty()) return false;
    colorMat = tempColor.clone();

    if (!m_depthStream.isValid()) return false;
    openni::Status rc = m_depthStream.readFrame(&m_depthFrame);
    if (rc != openni::STATUS_OK || !m_depthFrame.isValid()) return false;

    const openni::DepthPixel* pDepth = static_cast<const openni::DepthPixel*>(m_depthFrame.getData());
    if (!pDepth) return false;

    depthMat = cv::Mat(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
    memcpy(depthMat.data, pDepth, depthMat.total() * sizeof(uint16_t));

    return !depthMat.empty();
}

bool AstraCamera::getIntrinsics(float &fx, float &fy, float &cx, float &cy) const
{
    fx = m_fx; fy = m_fy; cx = m_cx; cy = m_cy;
    return true;
}

void AstraCamera::generateTestFrames(cv::Mat &color, cv::Mat &depth)
{
    static cv::Mat testColor = [](){
        cv::Mat img(480, 640, CV_8UC3);
        cv::randu(img, cv::Scalar(0,0,0), cv::Scalar(255,255,255));
        cv::putText(img, "EMULATION", cv::Point(200,240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
        return img;
    }();
    static cv::Mat testDepth = [](){
        cv::Mat img(480, 640, CV_16UC1);
        cv::randu(img, 500, 2000);
        return img;
    }();
    color = testColor.clone();
    depth = testDepth.clone();
    QThread::msleep(33);
}
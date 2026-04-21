#ifndef CAPTUREWORKER_H
#define CAPTUREWORKER_H

#include <QObject>
#include <QSharedPointer>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cv {
    class Mat;
}

class CaptureWorker : public QObject
{
    Q_OBJECT
public:
    explicit CaptureWorker(QObject *parent = nullptr);
    ~CaptureWorker();

    void setCloudProcessingEnabled(bool enabled) { m_cloudProcessingEnabled = enabled; }

public slots:
    void process();
    void stop();

signals:
    void frameCaptured(QSharedPointer<cv::Mat> color, QSharedPointer<cv::Mat> depth);
    void pointCloudReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void error(const QString &message);
    // Эмитится на каждый успешно прочитанный кадр с камеры. Используется GUI
    // для точного подсчёта кадров и FPS (frameCaptured дросселируется).
    void frameProcessed(int totalFrames);
    void warning(const QString &message);
    void finished();

private:
    bool m_running = true;
    bool m_cloudProcessingEnabled = true;
    // Интринсики, *уже* приведённые к разрешению depth-кадра, с которым
    // работает convertToPointCloud. scaleIntrinsicsToDepth() делает пересчёт.
    float m_fx = 570.0f, m_fy = 570.0f, m_cx = 320.0f, m_cy = 240.0f;
    // Разрешение, на котором была откалибрована RGB-камера. Берётся из
    // data/camera_calibration.xml (image_size), либо остаётся 0 если
    // использованы default-интринсики.
    int m_calibWidth = 0;
    int m_calibHeight = 0;
    bool m_intrinsicsLoaded = false;
    bool m_firstPointLogged = false;

    // Масштабирование интринсик с разрешения калибровки (m_calibWidth x
    // m_calibHeight) на разрешение depth-кадра (depthWidth x depthHeight).
    void scaleIntrinsicsToDepth(int depthWidth, int depthHeight);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPointCloud(
        const cv::Mat &depth, const cv::Mat &color,
        float fx, float fy, float cx, float cy);
};

#endif // CAPTUREWORKER_H
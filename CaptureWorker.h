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
    void finished();

private:
    bool m_running = true;
    bool m_cloudProcessingEnabled = true;
    float m_fx = 570.0f, m_fy = 570.0f, m_cx = 320.0f, m_cy = 240.0f;
    bool m_intrinsicsLoaded = false;
    bool m_firstPointLogged = false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPointCloud(
        const cv::Mat &depth, const cv::Mat &color,
        float fx, float fy, float cx, float cy);
};

#endif // CAPTUREWORKER_H
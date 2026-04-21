#include "CaptureWorker.h"
#include "AstraCamera.h"
#include <QThread>
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QSharedPointer>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <cmath>

CaptureWorker::CaptureWorker(QObject *parent) : QObject(parent) {}
CaptureWorker::~CaptureWorker() {}

void CaptureWorker::process()
{
    qDebug() << "[Worker] Thread started:" << QThread::currentThread();
    m_firstPointLogged = false;

    AstraCamera camera;
    // initialize() всегда возвращает true — если железо недоступно, камера
    // переходит в эмуляцию и сообщает об этом через isEmulationActive().
    camera.initialize();
    if (camera.isEmulationActive()) {
        const QString reason = QString::fromStdString(camera.getLastError());
        qWarning() << "[Worker] Camera emulation is active:" << reason;
        emit warning(QString("Камера недоступна, работает эмуляция. %1").arg(reason));
    }
    qInfo() << "[Worker] Camera initialized";

    cv::FileStorage fs("data/camera_calibration.xml", cv::FileStorage::READ);
    if (fs.isOpened()) {
        cv::Mat K;
        fs["camera_matrix"] >> K;
        if (!K.empty() && K.cols == 3 && K.rows == 3) {
            // OpenCV сохраняет calibrateCamera() в CV_64F, но файл может быть
            // подправлен вручную. Приводим к double независимо от исходного типа.
            cv::Mat K64;
            K.convertTo(K64, CV_64F);
            m_fx = static_cast<float>(K64.at<double>(0,0));
            m_fy = static_cast<float>(K64.at<double>(1,1));
            m_cx = static_cast<float>(K64.at<double>(0,2));
            m_cy = static_cast<float>(K64.at<double>(1,2));
            m_intrinsicsLoaded = true;
            qInfo() << "[Worker] Loaded calibration: fx=" << m_fx << "fy=" << m_fy << "cx=" << m_cx << "cy=" << m_cy;
        }
        fs.release();
    } else {
        if (camera.getIntrinsics(m_fx, m_fy, m_cx, m_cy)) {
            m_intrinsicsLoaded = true;
            qInfo() << "[Worker] Camera intrinsics: fx=" << m_fx << "fy=" << m_fy << "cx=" << m_cx << "cy=" << m_cy;
        }
    }

    if (std::isnan(m_fx) || m_fx < 100) {
        m_fx = m_fy = 570.0f;
        m_cx = 320.0f; m_cy = 240.0f;
        qWarning() << "[Worker] Using default intrinsics";
    }

    if (!camera.startStreams()) {
        emit error(QString::fromStdString(camera.getLastError()));
        emit finished();
        return;
    }
    qInfo() << "[Worker] Streams started, cloud processing:" << m_cloudProcessingEnabled;

    cv::Mat color, depth;
    int frameCounter = 0;
    int cloudSendCounter = 0;
    while (m_running) {
        if (camera.readFrame(color, depth) && !color.empty() && !depth.empty()) {
            if (!m_cloudProcessingEnabled && frameCounter % 10 == 0) {
                auto colorPtr = QSharedPointer<cv::Mat>::create(color.clone());
                auto depthPtr = QSharedPointer<cv::Mat>::create(depth.clone());
                emit frameCaptured(colorPtr, depthPtr);
            }

            if (m_cloudProcessingEnabled) {
                auto cloud = convertToPointCloud(depth, color, m_fx, m_fy, m_cx, m_cy);
                if (cloud && !cloud->empty()) {
                    if (++cloudSendCounter % 3 == 0) {
                        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
                        voxel.setInputCloud(cloud);
                        voxel.setLeafSize(0.002f, 0.002f, 0.002f);
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
                        voxel.filter(*filtered);
                        emit pointCloudReady(filtered);
                    }
                }
            }

            ++frameCounter;
            emit frameProcessed(frameCounter);
            if (frameCounter % 30 == 0) {
                qDebug() << "[Worker] Processed" << frameCounter << "frames";
            }
        } else {
            QThread::msleep(1);
        }
    }

    qDebug() << "[Worker] Exiting loop";
    camera.stopStreams();
    camera.shutdown();
    emit finished();
    qDebug() << "[Worker] Finished";
}

void CaptureWorker::stop()
{
    qDebug() << "[Worker] Stop requested";
    m_running = false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CaptureWorker::convertToPointCloud(
    const cv::Mat &depth, const cv::Mat &color,
    float fx, float fy, float cx, float cy)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (depth.empty() || color.empty()) return cloud;

    cv::Mat colorResized;
    if (color.cols != depth.cols || color.rows != depth.rows) {
        cv::resize(color, colorResized, cv::Size(depth.cols, depth.rows));
    } else {
        colorResized = color;
    }

    int width = depth.cols, height = depth.rows;
    int stride = 3;

    const uint16_t* depthPtr = depth.ptr<uint16_t>();
    const cv::Vec3b* colorPtr = colorResized.ptr<cv::Vec3b>();

    int validPoints = 0;
    for (int v = 0; v < height; v += stride) {
        for (int u = 0; u < width; u += stride) {
            uint16_t d = depthPtr[v * width + u];
            if (d == 0) continue;

            float z = d * 0.001f;
            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            pcl::PointXYZRGB pt;
            pt.x = x; pt.y = y; pt.z = z;
            const cv::Vec3b& bgr = colorPtr[v * width + u];
            pt.r = bgr[2]; pt.g = bgr[1]; pt.b = bgr[0];
            cloud->push_back(pt);
            validPoints++;
        }
    }

    if (!m_firstPointLogged && validPoints > 0) {
        qDebug() << "[Worker] First point generated";
        m_firstPointLogged = true;
    }
    return cloud;
}
#include "CameraCalibrator.h"
#include <QDebug>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>

CameraCalibrator::CameraCalibrator(QObject *parent) : QObject(parent) {}

bool CameraCalibrator::addFrame(const cv::Mat &image)
{
    if (image.empty()) return false;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, m_boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        m_imagePoints.push_back(corners);
        if (m_imageSize.empty()) {
            m_imageSize = image.size();
        }

        emit frameAdded(static_cast<int>(m_imagePoints.size()));
        emit statusChanged(QString("Кадр %1 добавлен").arg(m_imagePoints.size()));
        return true;
    } else {
        emit statusChanged("Шахматная доска не найдена");
        return false;
    }
}

bool CameraCalibrator::calibrate()
{
    if (m_imagePoints.size() < 5) {
        emit statusChanged("Нужно минимум 5 кадров");
        return false;
    }

    emit statusChanged("Калибровка, подождите...");

    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    for (int i = 0; i < m_boardSize.height; ++i) {
        for (int j = 0; j < m_boardSize.width; ++j) {
            objectPoints[0].push_back(cv::Point3f(j * m_squareSize / 1000.0f, i * m_squareSize / 1000.0f, 0.0f));
        }
    }
    objectPoints.resize(m_imagePoints.size(), objectPoints[0]);

    m_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    m_distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(objectPoints, m_imagePoints, m_imageSize,
        m_cameraMatrix, m_distCoeffs, rvecs, tvecs,
        cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);

    m_calibrated = true;
    emit statusChanged(QString("Калибровка завершена. Ошибка RMS: %1").arg(rms));
    return true;
}

bool CameraCalibrator::saveToFile(const std::string &filename)
{
    namespace fs = std::filesystem;
    fs::path filePath(filename);
    auto parentPath = filePath.parent_path();
    if (!parentPath.empty() && !fs::exists(parentPath)) {
        std::error_code ec;
        if (!fs::create_directories(parentPath, ec)) {
            emit statusChanged(QString("Не удалось создать папку: %1").arg(QString::fromStdString(ec.message())));
            return false;
        }
    }

    cv::FileStorage fsStorage(filename, cv::FileStorage::WRITE);
    if (!fsStorage.isOpened()) {
        emit statusChanged(QString("Ошибка открытия файла %1 для записи").arg(QString::fromStdString(filename)));
        return false;
    }
    fsStorage << "camera_matrix" << m_cameraMatrix;
    fsStorage << "distortion_coefficients" << m_distCoeffs;
    fsStorage << "image_size" << m_imageSize;
    fsStorage.release();

    emit statusChanged(QString("Калибровка сохранена в %1").arg(QString::fromStdString(fs::absolute(filePath).string())));
    return true;
}

bool CameraCalibrator::loadFromFile(const std::string &filename)
{
    cv::FileStorage fsStorage(filename, cv::FileStorage::READ);
    if (!fsStorage.isOpened()) return false;
    fsStorage["camera_matrix"] >> m_cameraMatrix;
    fsStorage["distortion_coefficients"] >> m_distCoeffs;
    fsStorage["image_size"] >> m_imageSize;
    m_calibrated = true;
    fsStorage.release();
    return true;
}

void CameraCalibrator::reset()
{
    m_imagePoints.clear();
    m_calibrated = false;
    m_cameraMatrix.release();
    m_distCoeffs.release();
    emit statusChanged("Сброшено");
}
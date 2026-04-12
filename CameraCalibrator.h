#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <QObject>
#include <opencv2/opencv.hpp>

class CameraCalibrator : public QObject
{
    Q_OBJECT
public:
    explicit CameraCalibrator(QObject *parent = nullptr);

    void setBoardSize(int width, int height) { m_boardSize = cv::Size(width, height); }
    void setSquareSize(float sizeMM) { m_squareSize = sizeMM; }

    bool addFrame(const cv::Mat &image);
    bool calibrate();

    cv::Mat getCameraMatrix() const { return m_cameraMatrix; }
    cv::Mat getDistCoeffs() const { return m_distCoeffs; }

    bool saveToFile(const std::string &filename);
    bool loadFromFile(const std::string &filename);

    void reset();

    int getFrameCount() const { return m_imagePoints.size(); }
    bool isCalibrated() const { return m_calibrated; }

signals:
    void statusChanged(const QString &msg);
    void frameAdded(int count);

private:
    cv::Size m_boardSize{9, 6};
    float m_squareSize = 25.0f;

    std::vector<std::vector<cv::Point2f>> m_imagePoints;
    cv::Size m_imageSize;

    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
    bool m_calibrated = false;
};

#endif // CAMERACALIBRATOR_H
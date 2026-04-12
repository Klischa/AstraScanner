#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QProgressBar>
#include <QElapsedTimer>
#include <QThread>
#include <QTimer>
#include <QTextEdit>
#include <QMutex>
#include <QSharedPointer>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
class QTabWidget;
class QPushButton;
class QVTKOpenGLNativeWidget;
QT_END_NAMESPACE

class CaptureWorker;
class CameraCalibrator;
class LiveCloudWindow;
class PointCloudFilters;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    Q_INVOKABLE void appendLog(const QString &text);   // <-- добавлено Q_INVOKABLE

signals:
    void cloudSizeChanged(int size);
    void distanceChanged(int distance);

private slots:
    void onPreviewClicked();
    void onScanClicked();
    void onPauseClicked();
    void onStopClicked();
    void onClearClicked();
    void onNewFrame(QSharedPointer<cv::Mat> color, QSharedPointer<cv::Mat> depth);
    void onPointCloudReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void onCaptureError(const QString &msg);
    void updateViewer();
    void checkScanTimeout();

    void onCalibPreviewClicked();
    void onCalibCaptureClicked();
    void onCalibCalibrateClicked();
    void onCalibResetClicked();
    void onCalibrationStatus(const QString &msg);

private:
    void setupUI();
    void setupVisualizer();
    void startCapture(bool enableCloudProcessing);
    void stopCapture();

    QTabWidget *m_tabWidget = nullptr;

    QPushButton *m_previewBtn = nullptr;
    QPushButton *m_scanBtn = nullptr;
    QPushButton *m_pauseBtn = nullptr;
    QPushButton *m_stopBtn = nullptr;
    QPushButton *m_clearBtn = nullptr;
    QLabel *m_rgbLabel = nullptr;
    QLabel *m_depthLabel = nullptr;
    QProgressBar *m_distanceIndicator = nullptr;
    QLabel *m_fpsLabel = nullptr;
    QLabel *m_frameCountLabel = nullptr;
    QLabel *m_timeLabel = nullptr;

    QVTKOpenGLNativeWidget *m_vtkWidget = nullptr;
    pcl::visualization::PCLVisualizer::Ptr m_viewer;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_accumulatedCloud;
    QMutex m_cloudMutex;

    bool m_scanning = false;
    bool m_cloudProcessing = false;
    int m_frameSkip = 5;
    int m_frameCounter = 0;
    QTimer *m_viewerUpdateTimer = nullptr;
    QTimer *m_scanTimeoutTimer = nullptr;

    QThread *m_captureThread = nullptr;
    CaptureWorker *m_worker = nullptr;

    int m_totalFrames = 0;
    QElapsedTimer m_scanTimer;

    cv::Mat m_lastColorFrame;
    int m_depthColormap = cv::COLORMAP_JET;

    CameraCalibrator *m_calibrator = nullptr;
    QLabel *m_calibStatusLabel = nullptr;
    QLabel *m_calibFrameCountLabel = nullptr;
    QPushButton *m_calibPreviewBtn = nullptr;
    QPushButton *m_calibCaptureBtn = nullptr;
    QPushButton *m_calibCalibrateBtn = nullptr;
    QPushButton *m_calibResetBtn = nullptr;
    QLabel *m_calibRgbLabel = nullptr;

    LiveCloudWindow *m_liveCloudWindow = nullptr;
    PointCloudFilters *m_filters = nullptr;

    QTextEdit *m_logTextEdit = nullptr;
};

#endif // MAINWINDOW_H
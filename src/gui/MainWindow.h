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
#include <atomic>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PolygonMesh.h>
#include <QFutureWatcher>
#include <opencv2/opencv.hpp>

#include "../filters/PointCloudFilters.h"

QT_BEGIN_NAMESPACE
class QTabWidget;
class QPushButton;
class QVTKOpenGLNativeWidget;
class QCheckBox;
class QSpinBox;
class QDoubleSpinBox;
QT_END_NAMESPACE

class CaptureWorker;
class CameraCalibrator;
class LiveCloudWindow;
class PointCloudFilters;
class ProjectManager;
class ExportManager;

QT_BEGIN_NAMESPACE
class QListWidget;
class QListWidgetItem;
QT_END_NAMESPACE

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
    void onCaptureWarning(const QString &msg);
    void onFrameProcessed(int totalFrames);
    void updateViewer();
    void checkScanTimeout();

    void onCalibPreviewClicked();
    void onCalibCaptureClicked();
    void onCalibCalibrateClicked();
    void onCalibResetClicked();
    void onCalibrationStatus(const QString &msg);

    // --- Проект / экспорт ---
    void onNewProject();
    void onOpenProject();
    void onSaveProject();
    void onSaveProjectAs();
    void onAddCurrentCloudToProject();
    void onExportCurrentCloud();
    void onExportMesh();
    void onScansListContextMenu(const QPoint &pos);
    void onScansListDoubleClicked(QListWidgetItem *item);
    void refreshScansList();

    // --- Poisson-реконструкция ---
    void onReconstructMeshClicked(const PointCloudFilters::PoissonParams &params);
    void onShowCloudClicked();
    void onPoissonFinished();

    // --- ICP-регистрация ---
    void onMergeScansClicked(const PointCloudFilters::MergeParams &params);
    void onMergeFinished();
    void onSaveMergedToProject();

    // --- Настройки (единый диалог) ---
    void onShowSettingsDialog();

    // --- Поворотный стол / авто-сохранение по таймеру ---
    void onTurntableToggled(bool enabled);
    void onTurntableTick();

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

    // --- Проект / экспорт ---
    ProjectManager *m_project = nullptr;
    ExportManager *m_exporter = nullptr;
    QListWidget *m_scansList = nullptr;
    QLabel *m_projectStatusLabel = nullptr;
    // Последний реконструированный меш (Poisson). Пустой, пока не нажата
    // кнопка «Построить меш» в вкладке «Обработка» или до первой успешной
    // реконструкции.
    pcl::PolygonMesh m_lastMesh;
    QProgressBar *m_poissonProgress = nullptr;
    QLabel *m_meshStatusLabel = nullptr;
    // Идентификатор меша в PCLVisualizer. Пустая строка = меш не отображён.
    QString m_meshViewerId;
    // Watcher для QtConcurrent::run — чтобы не потерять future и корректно
    // эмитить onPoissonFinished() в GUI-потоке.
    QFutureWatcher<pcl::PolygonMesh> *m_poissonWatcher = nullptr;

    // ICP-регистрация
    QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *m_mergeWatcher = nullptr;
    QPushButton *m_mergeBtn = nullptr;
    QPushButton *m_addMergedBtn = nullptr;
    QLabel *m_icpStatusLabel = nullptr;
    // Последний результат ICP-мержа, готовый к записи в проект как новый
    // скан. Null — результата ещё нет.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_lastMerged;

    // Ручная переориентация нормалей для Poisson. Живут прямо в UI
    // группы «Реконструкция поверхности», чтобы значения читались в момент
    // нажатия «Построить меш».
    QCheckBox       *m_poissonFlipNormalsChk = nullptr;
    QCheckBox       *m_poissonCustomVpChk = nullptr;
    QDoubleSpinBox  *m_poissonVpX = nullptr;
    QDoubleSpinBox  *m_poissonVpY = nullptr;
    QDoubleSpinBox  *m_poissonVpZ = nullptr;
    QCheckBox       *m_poissonConsistentOrientChk = nullptr;

    // Поворотный стол / авто-сохранение
    QTimer          *m_turntableTimer = nullptr;
    QCheckBox       *m_turntableEnableChk = nullptr;
    QSpinBox        *m_turntableIntervalSpin = nullptr;
    QSpinBox        *m_turntableCountSpin = nullptr;
    QLabel          *m_turntableStatusLabel = nullptr;
    int              m_turntableCaptured = 0;
};

// Глобальный указатель на главное окно, используется обработчиком сообщений
// Qt (fileMessageHandler) для форвардинга логов в GUI. Доступ атомарный,
// потому что сообщения могут приходить из любого потока.
extern std::atomic<MainWindow *> g_mainWindow;

#endif // MAINWINDOW_H
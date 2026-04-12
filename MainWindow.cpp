#include "MainWindow.h"
#include "../capture/CaptureWorker.h"
#include "../calibration/CameraCalibrator.h"
#include "LiveCloudWindow.h"
#include "../filters/PointCloudFilters.h"

#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QStatusBar>
#include <QElapsedTimer>
#include <QTextEdit>
#include <QScrollBar>
#include <QComboBox>
#include <QMessageBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <vtk-9.3/QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <QDebug>
#include <filesystem>

MainWindow* g_mainWindow = nullptr;

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_accumulatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    g_mainWindow = this;

    std::error_code ec;
    std::filesystem::create_directories("data", ec);

    m_liveCloudWindow = new LiveCloudWindow(this);
    m_filters = new PointCloudFilters(this);

    setupUI();
    setupVisualizer();

    m_viewerUpdateTimer = new QTimer(this);
    connect(m_viewerUpdateTimer, &QTimer::timeout, this, &MainWindow::updateViewer);
    m_viewerUpdateTimer->start(1000);

    m_scanTimeoutTimer = new QTimer(this);
    m_scanTimeoutTimer->setSingleShot(true);
    connect(m_scanTimeoutTimer, &QTimer::timeout, this, &MainWindow::checkScanTimeout);

    resize(1400, 900);

    m_calibrator = new CameraCalibrator(this);
    m_calibrator->setBoardSize(9, 6);
    m_calibrator->setSquareSize(25.0f);
    connect(m_calibrator, &CameraCalibrator::statusChanged, this, &MainWindow::onCalibrationStatus);
    connect(m_calibrator, &CameraCalibrator::frameAdded, this, [this](int count) {
        m_calibFrameCountLabel->setText(QString("Кадров: %1 / 12").arg(count));
        m_calibCalibrateBtn->setEnabled(count >= 5);
    });

    if (m_calibrator->loadFromFile("data/camera_calibration.xml")) {
        m_calibStatusLabel->setText("Калибровка загружена");
    }

    qInfo() << "Application started";
}

MainWindow::~MainWindow()
{
    stopCapture();
    if (m_viewer) {
        m_viewer->close();
    }
    g_mainWindow = nullptr;
}

void MainWindow::appendLog(const QString &text)
{
    if (m_logTextEdit) {
        m_logTextEdit->append(text);
        QScrollBar *sb = m_logTextEdit->verticalScrollBar();
        sb->setValue(sb->maximum());
    }
}

void MainWindow::setupUI()
{
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);

    m_tabWidget = new QTabWidget(this);
    mainLayout->addWidget(m_tabWidget);

    // Вкладка "Главная"
    QWidget* homeTab = new QWidget();
    m_tabWidget->addTab(homeTab, "Главная");
    QVBoxLayout* homeLayout = new QVBoxLayout(homeTab);

    QGroupBox* cameraGroup = new QGroupBox("Информация о камере", this);
    QVBoxLayout* cameraLayout = new QVBoxLayout(cameraGroup);

    QLabel* cameraStatusLabel = new QLabel("Камера: не инициализирована", this);
    QLabel* intrinsicsLabel = new QLabel("Интринсики: не загружены", this);
    QLabel* calibrationLabel = new QLabel("Калибровка: не загружена", this);

    cameraLayout->addWidget(cameraStatusLabel);
    cameraLayout->addWidget(intrinsicsLabel);
    cameraLayout->addWidget(calibrationLabel);

    homeLayout->addWidget(cameraGroup);

    QGroupBox* settingsGroup = new QGroupBox("Настройки отображения", this);
    QVBoxLayout* settingsLayout = new QVBoxLayout(settingsGroup);

    QHBoxLayout* colormapLayout = new QHBoxLayout();
    QLabel* colormapLabel = new QLabel("Цветовая карта глубины:", this);
    QComboBox* colormapCombo = new QComboBox(this);
    colormapCombo->addItem("Jet", cv::COLORMAP_JET);
    colormapCombo->addItem("Hot", cv::COLORMAP_HOT);
    colormapCombo->addItem("Rainbow", cv::COLORMAP_RAINBOW);
    colormapCombo->addItem("Grayscale", cv::COLORMAP_BONE);
    QPushButton* applyColormapBtn = new QPushButton("Применить", this);
    colormapLayout->addWidget(colormapLabel);
    colormapLayout->addWidget(colormapCombo);
    colormapLayout->addWidget(applyColormapBtn);
    colormapLayout->addStretch();
    settingsLayout->addLayout(colormapLayout);

    QHBoxLayout* rangeLayout = new QHBoxLayout();
    QLabel* minRangeLabel = new QLabel("Мин. расстояние (мм):", this);
    QSpinBox* minRangeSpin = new QSpinBox(this);
    minRangeSpin->setRange(0, 2000);
    minRangeSpin->setValue(0);
    QLabel* maxRangeLabel = new QLabel("Макс. расстояние (мм):", this);
    QSpinBox* maxRangeSpin = new QSpinBox(this);
    maxRangeSpin->setRange(500, 5000);
    maxRangeSpin->setValue(2000);
    rangeLayout->addWidget(minRangeLabel);
    rangeLayout->addWidget(minRangeSpin);
    rangeLayout->addWidget(maxRangeLabel);
    rangeLayout->addWidget(maxRangeSpin);
    rangeLayout->addStretch();
    settingsLayout->addLayout(rangeLayout);

    QCheckBox* smoothingCheck = new QCheckBox("Сглаживание глубины", this);
    settingsLayout->addWidget(smoothingCheck);

    QCheckBox* invertCheck = new QCheckBox("Инвертировать цвета", this);
    settingsLayout->addWidget(invertCheck);

    homeLayout->addWidget(settingsGroup);

    QGroupBox* distanceGroup = new QGroupBox("Индикатор расстояния", this);
    QVBoxLayout* distanceLayout = new QVBoxLayout(distanceGroup);

    m_distanceIndicator = new QProgressBar(this);
    m_distanceIndicator->setRange(0, 2000);
    m_distanceIndicator->setValue(0);
    m_distanceIndicator->setFormat("Расстояние: %v мм");
    distanceLayout->addWidget(m_distanceIndicator);

    QLabel* distanceHintLabel = new QLabel("Зелёный: оптимально | Оранжевый: допустимо | Красный: скорректируйте", this);
    distanceHintLabel->setStyleSheet("font-size: 10px; color: gray;");
    distanceLayout->addWidget(distanceHintLabel);

    homeLayout->addWidget(distanceGroup);
    
    QPushButton *liveCloudBtn = new QPushButton("📊 Live Cloud Viewer", this);
    liveCloudBtn->setCheckable(true);
    liveCloudBtn->setMaximumWidth(200);
    homeLayout->addWidget(liveCloudBtn, 0, Qt::AlignHCenter);

    connect(liveCloudBtn, &QPushButton::toggled, this, [this, liveCloudBtn](bool checked) {
        if (checked) {
            m_liveCloudWindow->show();
            m_liveCloudWindow->startUpdating();
            liveCloudBtn->setText("📊 Live Cloud Viewer (ON)");
        } else {
            m_liveCloudWindow->stopUpdating();
            m_liveCloudWindow->hide();
            liveCloudBtn->setText("📊 Live Cloud Viewer");
        }
    });

    connect(m_liveCloudWindow, &LiveCloudWindow::cloudRequested, this, [this]() {
        QMutexLocker locker(&m_cloudMutex);
        if (m_accumulatedCloud && !m_accumulatedCloud->empty()) {
            m_liveCloudWindow->updateCloud(m_accumulatedCloud);
        }
    });
    
    homeLayout->addStretch();

    connect(applyColormapBtn, &QPushButton::clicked, this, [this, colormapCombo]() {
        m_depthColormap = colormapCombo->currentData().toInt();
        statusBar()->showMessage("Цветовая карта применена", 2000);
    });

    // Вкладка "Сканирование"
    QWidget* scanTab = new QWidget();
    m_tabWidget->addTab(scanTab, "Сканирование");
    QVBoxLayout* scanLayout = new QVBoxLayout(scanTab);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_previewBtn = new QPushButton("Preview", this);
    m_scanBtn = new QPushButton("Scan", this);
    m_pauseBtn = new QPushButton("Pause", this);
    m_stopBtn = new QPushButton("Stop", this);
    m_clearBtn = new QPushButton("Clear", this);
    m_pauseBtn->setEnabled(false);
    m_stopBtn->setEnabled(false);
    buttonLayout->addWidget(m_previewBtn);
    buttonLayout->addWidget(m_scanBtn);
    buttonLayout->addWidget(m_pauseBtn);
    buttonLayout->addWidget(m_stopBtn);
    buttonLayout->addWidget(m_clearBtn);
    buttonLayout->addStretch();
    scanLayout->addLayout(buttonLayout);

    QHBoxLayout* videoLayout = new QHBoxLayout();
    m_rgbLabel = new QLabel("RGB Stream", this);
    m_rgbLabel->setMinimumSize(640, 480);
    m_rgbLabel->setStyleSheet("border: 1px solid gray; background-color: black;");
    m_rgbLabel->setAlignment(Qt::AlignCenter);
    m_depthLabel = new QLabel("Depth Stream", this);
    m_depthLabel->setMinimumSize(640, 480);
    m_depthLabel->setStyleSheet("border: 1px solid gray; background-color: black;");
    m_depthLabel->setAlignment(Qt::AlignCenter);
    videoLayout->addWidget(m_rgbLabel);
    videoLayout->addWidget(m_depthLabel);
    scanLayout->addLayout(videoLayout);

    m_distanceIndicator = new QProgressBar(this);
    m_distanceIndicator->setRange(0, 2000);
    m_distanceIndicator->setValue(0);
    m_distanceIndicator->setFormat("Distance: %v mm");
    scanLayout->addWidget(m_distanceIndicator);

    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    m_vtkWidget->setMinimumSize(640, 480);
    scanLayout->addWidget(m_vtkWidget);
    scanLayout->addStretch();

    // Вкладка "Калибровка"
    QWidget* calibTab = new QWidget();
    m_tabWidget->addTab(calibTab, "Калибровка");
    QVBoxLayout* calibLayout = new QVBoxLayout(calibTab);

    QHBoxLayout* calibBtnLayout = new QHBoxLayout();
    m_calibPreviewBtn = new QPushButton("Preview", this);
    m_calibCaptureBtn = new QPushButton("Захватить кадр", this);
    m_calibCalibrateBtn = new QPushButton("Калибровать", this);
    m_calibResetBtn = new QPushButton("Сброс", this);
    m_calibCalibrateBtn->setEnabled(false);
    calibBtnLayout->addWidget(m_calibPreviewBtn);
    calibBtnLayout->addWidget(m_calibCaptureBtn);
    calibBtnLayout->addWidget(m_calibCalibrateBtn);
    calibBtnLayout->addWidget(m_calibResetBtn);
    calibBtnLayout->addStretch();
    calibLayout->addLayout(calibBtnLayout);

    m_calibRgbLabel = new QLabel("RGB кадр", this);
    m_calibRgbLabel->setMinimumSize(640, 480);
    m_calibRgbLabel->setStyleSheet("border: 1px solid gray; background-color: black;");
    m_calibRgbLabel->setAlignment(Qt::AlignCenter);
    calibLayout->addWidget(m_calibRgbLabel);

    m_calibFrameCountLabel = new QLabel("Кадров: 0 / 12", this);
    m_calibStatusLabel = new QLabel("Нажмите Preview для начала", this);
    calibLayout->addWidget(m_calibFrameCountLabel);
    calibLayout->addWidget(m_calibStatusLabel);
    calibLayout->addStretch();

    connect(m_calibPreviewBtn, &QPushButton::clicked, this, &MainWindow::onCalibPreviewClicked);
    connect(m_calibCaptureBtn, &QPushButton::clicked, this, &MainWindow::onCalibCaptureClicked);
    connect(m_calibCalibrateBtn, &QPushButton::clicked, this, &MainWindow::onCalibCalibrateClicked);
    connect(m_calibResetBtn, &QPushButton::clicked, this, &MainWindow::onCalibResetClicked);

    // Вкладка "Обработка"
    QWidget* processingTab = new QWidget();
    m_tabWidget->addTab(processingTab, "Обработка");
    QVBoxLayout* processingLayout = new QVBoxLayout(processingTab);

    QGroupBox* filterGroup = new QGroupBox("Фильтры очистки", this);
    QVBoxLayout* filterLayout = new QVBoxLayout(filterGroup);

    QHBoxLayout* sorLayout = new QHBoxLayout();
    QLabel* sorLabel = new QLabel("SOR (mean K):", this);
    QSpinBox* sorMeanKSpin = new QSpinBox(this);
    sorMeanKSpin->setRange(10, 100);
    sorMeanKSpin->setValue(50);
    QLabel* sorThreshLabel = new QLabel("порог:", this);
    QDoubleSpinBox* sorThreshSpin = new QDoubleSpinBox(this);
    sorThreshSpin->setRange(0.1, 5.0);
    sorThreshSpin->setSingleStep(0.1);
    sorThreshSpin->setValue(1.0);
    QPushButton* sorBtn = new QPushButton("Применить SOR", this);
    sorLayout->addWidget(sorLabel);
    sorLayout->addWidget(sorMeanKSpin);
    sorLayout->addWidget(sorThreshLabel);
    sorLayout->addWidget(sorThreshSpin);
    sorLayout->addWidget(sorBtn);
    sorLayout->addStretch();
    filterLayout->addLayout(sorLayout);

    QHBoxLayout* rorLayout = new QHBoxLayout();
    QLabel* rorLabel = new QLabel("ROR (радиус):", this);
    QDoubleSpinBox* rorRadiusSpin = new QDoubleSpinBox(this);
    rorRadiusSpin->setRange(0.001, 0.1);
    rorRadiusSpin->setSingleStep(0.001);
    rorRadiusSpin->setValue(0.02);
    QLabel* rorNeighborsLabel = new QLabel("мин. соседей:", this);
    QSpinBox* rorNeighborsSpin = new QSpinBox(this);
    rorNeighborsSpin->setRange(1, 50);
    rorNeighborsSpin->setValue(10);
    QPushButton* rorBtn = new QPushButton("Применить ROR", this);
    rorLayout->addWidget(rorLabel);
    rorLayout->addWidget(rorRadiusSpin);
    rorLayout->addWidget(rorNeighborsLabel);
    rorLayout->addWidget(rorNeighborsSpin);
    rorLayout->addWidget(rorBtn);
    rorLayout->addStretch();
    filterLayout->addLayout(rorLayout);

    QHBoxLayout* voxelLayout = new QHBoxLayout();
    QLabel* voxelLabel = new QLabel("Воксель (размер):", this);
    QDoubleSpinBox* voxelSizeSpin = new QDoubleSpinBox(this);
    voxelSizeSpin->setRange(0.001, 0.05);
    voxelSizeSpin->setSingleStep(0.001);
    voxelSizeSpin->setValue(0.005);
    QPushButton* voxelBtn = new QPushButton("Применить воксель", this);
    voxelLayout->addWidget(voxelLabel);
    voxelLayout->addWidget(voxelSizeSpin);
    voxelLayout->addWidget(voxelBtn);
    voxelLayout->addStretch();
    filterLayout->addLayout(voxelLayout);

    QPushButton* magicWandBtn = new QPushButton("Magic Wand (быстрая очистка)", this);
    filterLayout->addWidget(magicWandBtn);

    processingLayout->addWidget(filterGroup);

    QGroupBox* registrationGroup = new QGroupBox("Регистрация сканов", this);
    QVBoxLayout* registrationLayout = new QVBoxLayout(registrationGroup);

    QPushButton* mergeScansBtn = new QPushButton("Объединить все сканы", this);
    registrationLayout->addWidget(mergeScansBtn);

    QProgressBar* registrationProgress = new QProgressBar(this);
    registrationProgress->setRange(0, 100);
    registrationProgress->setValue(0);
    registrationLayout->addWidget(registrationProgress);

    processingLayout->addWidget(registrationGroup);

    connect(sorBtn, &QPushButton::clicked, this, [this, sorMeanKSpin, sorThreshSpin]() {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) return;

        auto filtered = m_filters->applyStatisticalOutlierRemoval(
            m_accumulatedCloud, sorMeanKSpin->value(), sorThreshSpin->value());
        *m_accumulatedCloud = *filtered;
        emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
        updateViewer();
    });

    connect(rorBtn, &QPushButton::clicked, this, [this, rorRadiusSpin, rorNeighborsSpin]() {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) return;

        auto filtered = m_filters->applyRadiusOutlierRemoval(
            m_accumulatedCloud, rorRadiusSpin->value(), rorNeighborsSpin->value());
        *m_accumulatedCloud = *filtered;
        emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
        updateViewer();
    });

    connect(voxelBtn, &QPushButton::clicked, this, [this, voxelSizeSpin]() {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) return;

        auto filtered = m_filters->applyVoxelGrid(m_accumulatedCloud, voxelSizeSpin->value());
        *m_accumulatedCloud = *filtered;
        emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
        updateViewer();
    });

    connect(magicWandBtn, &QPushButton::clicked, this, [this]() {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) return;

        auto filtered = m_filters->applyMagicWand(m_accumulatedCloud);
        *m_accumulatedCloud = *filtered;
        emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
        updateViewer();
    });

    connect(mergeScansBtn, &QPushButton::clicked, this, [this, registrationProgress]() {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) return;

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> scans = {m_accumulatedCloud};
        auto merged = m_filters->mergeScans(scans);
        *m_accumulatedCloud = *merged;
        emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
        updateViewer();
    });

    connect(m_filters, &PointCloudFilters::progressUpdated, registrationProgress, &QProgressBar::setValue);
    connect(m_filters, &PointCloudFilters::filterCompleted, this, [this](const QString &filter, int before, int after) {
        statusBar()->showMessage(QString("%1: %2 -> %3 точек").arg(filter).arg(before).arg(after), 3000);
    });

    // Статусная строка
    m_fpsLabel = new QLabel("FPS: 0", this);
    m_frameCountLabel = new QLabel("Frames: 0", this);
    m_timeLabel = new QLabel("Time: 00:00", this);
    statusBar()->addPermanentWidget(m_fpsLabel);
    statusBar()->addPermanentWidget(m_frameCountLabel);
    statusBar()->addPermanentWidget(m_timeLabel);

    QLabel* cloudSizeLabel = new QLabel("Cloud: 0", this);
    statusBar()->addPermanentWidget(cloudSizeLabel);
    connect(this, &MainWindow::cloudSizeChanged, cloudSizeLabel, [cloudSizeLabel](int size) {
        cloudSizeLabel->setText(QString("Cloud: %1").arg(size));
    });

    connect(m_previewBtn, &QPushButton::clicked, this, &MainWindow::onPreviewClicked);
    connect(m_scanBtn, &QPushButton::clicked, this, &MainWindow::onScanClicked);
    connect(m_pauseBtn, &QPushButton::clicked, this, &MainWindow::onPauseClicked);
    connect(m_stopBtn, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(m_clearBtn, &QPushButton::clicked, this, &MainWindow::onClearClicked);
}

void MainWindow::setupVisualizer()
{
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = m_vtkWidget->renderWindow();
    renderWindow->AddRenderer(renderer);

    m_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    m_viewer->setupInteractor(m_vtkWidget->interactor(), renderWindow);
    m_viewer->setBackgroundColor(0.1, 0.1, 0.1);
    m_viewer->addCoordinateSystem(0.2);
    m_viewer->initCameraParameters();
    m_viewer->setCameraPosition(0, 0, -2, 0, 0, 1, 0, -1, 0);
    m_viewer->getRenderWindow()->SetDesiredUpdateRate(5.0);
    qDebug() << "Visualizer setup complete";
}

// ========== Основные кнопки ==========
void MainWindow::onPreviewClicked()
{
    qDebug() << "Preview clicked";
    if (m_captureThread) stopCapture();
    m_scanning = false;
    m_cloudProcessing = false;
    m_previewBtn->setEnabled(false);
    m_scanBtn->setEnabled(true);
    m_pauseBtn->setEnabled(false);
    m_stopBtn->setEnabled(true);
    m_totalFrames = 0;
    m_scanTimer.start();
    startCapture(false);
}

void MainWindow::onScanClicked()
{
    qDebug() << "Scan clicked";
    if (m_captureThread) stopCapture();
    m_scanning = true;
    m_cloudProcessing = true;
    m_previewBtn->setEnabled(false);
    m_scanBtn->setEnabled(false);
    m_pauseBtn->setEnabled(true);
    m_stopBtn->setEnabled(true);
    m_totalFrames = 0;
    m_frameCounter = 0;
    m_scanTimer.start();

    {
        QMutexLocker locker(&m_cloudMutex);
        if (m_accumulatedCloud) m_accumulatedCloud->clear();
    }
    if (m_viewer) {
        m_viewer->removeAllPointClouds();
        m_vtkWidget->renderWindow()->Render();
    }
    emit cloudSizeChanged(0);
    qDebug() << "Scan started";

    m_scanTimeoutTimer->start(5 * 60 * 1000);
    startCapture(true);
}

void MainWindow::checkScanTimeout()
{
    if (m_scanning) {
        qWarning() << "Scan timeout, stopping";
        onStopClicked();
    }
}

void MainWindow::onPauseClicked()
{
    m_scanning = !m_scanning;
    m_pauseBtn->setText(m_scanning ? "Pause" : "Resume");
}

void MainWindow::onStopClicked()
{
    qDebug() << "Stop clicked";
    m_scanTimeoutTimer->stop();
    m_scanning = false;
    m_cloudProcessing = false;
    stopCapture();
    m_previewBtn->setEnabled(true);
    m_scanBtn->setEnabled(true);
    m_pauseBtn->setEnabled(false);
    m_stopBtn->setEnabled(false);
    m_pauseBtn->setText("Pause");
}

void MainWindow::onClearClicked()
{
    qDebug() << "Clear clicked";
    {
        QMutexLocker locker(&m_cloudMutex);
        if (m_accumulatedCloud) m_accumulatedCloud->clear();
    }
    if (m_viewer) {
        m_viewer->removeAllPointClouds();
        m_vtkWidget->renderWindow()->Render();
    }
    m_totalFrames = 0;
    m_frameCounter = 0;
    m_frameCountLabel->setText("Frames: 0");
    emit cloudSizeChanged(0);
}

// ========== Управление потоком ==========
void MainWindow::startCapture(bool enableCloudProcessing)
{
    if (m_captureThread) stopCapture();
    m_captureThread = new QThread(this);
    m_worker = new CaptureWorker();
    m_worker->moveToThread(m_captureThread);
    m_worker->setCloudProcessingEnabled(enableCloudProcessing);

    connect(m_captureThread, &QThread::started, m_worker, &CaptureWorker::process);
    connect(m_worker, &CaptureWorker::frameCaptured, this, &MainWindow::onNewFrame);
    connect(m_worker, &CaptureWorker::pointCloudReady, this, &MainWindow::onPointCloudReady);
    connect(m_worker, &CaptureWorker::error, this, &MainWindow::onCaptureError);
    connect(m_worker, &CaptureWorker::finished, m_captureThread, &QThread::quit);
    connect(m_worker, &CaptureWorker::finished, m_worker, &QObject::deleteLater);
    connect(m_captureThread, &QThread::finished, m_captureThread, &QObject::deleteLater);

    m_captureThread->start();
    qDebug() << "Capture started, cloud:" << enableCloudProcessing;
}

void MainWindow::stopCapture()
{
    if (m_worker) {
        m_worker->stop();
        QThread::msleep(100);
    }
    if (m_captureThread) {
        m_captureThread->quit();
        if (!m_captureThread->wait(3000)) {
            m_captureThread->terminate();
            m_captureThread->wait();
        }
        m_captureThread = nullptr;
    }
    m_worker = nullptr;
    qDebug() << "Capture stopped";
}

// ========== Обработка кадров ==========
void MainWindow::onNewFrame(QSharedPointer<cv::Mat> color, QSharedPointer<cv::Mat> depth)
{
    static QElapsedTimer guiTimer;
    static int guiFrameCount = 0;
    if (!guiTimer.isValid()) guiTimer.start();

    // Обновляем GUI не чаще 10 раз в секунду
    bool updateGui = (guiTimer.elapsed() > 100);
    if (updateGui) {
        guiTimer.restart();
        guiFrameCount = 0;
    }

    if (color && !color->empty()) {
        m_lastColorFrame = color->clone();
        if (updateGui) {
            QImage qimg(color->data, color->cols, color->rows, color->step, QImage::Format_BGR888);
            QPixmap pix = QPixmap::fromImage(qimg).scaled(m_rgbLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
            m_rgbLabel->setPixmap(pix);
            if (m_calibRgbLabel) {
                m_calibRgbLabel->setPixmap(pix.scaled(m_calibRgbLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
            }
        }
    }

    if (depth && !depth->empty()) {
        if (updateGui) {
            double minVal, maxVal;
            cv::minMaxIdx(*depth, &minVal, &maxVal);
            cv::Mat depthNorm;
            depth->convertTo(depthNorm, CV_8UC1, 255.0 / (maxVal - minVal + 1e-6),
                             -minVal * 255.0 / (maxVal - minVal + 1e-6));
            cv::Mat depthColor;
            cv::applyColorMap(depthNorm, depthColor, m_depthColormap);
            QImage qimgDepth(depthColor.data, depthColor.cols, depthColor.rows,
                             depthColor.step, QImage::Format_BGR888);
            m_depthLabel->setPixmap(QPixmap::fromImage(qimgDepth).scaled(
                m_depthLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation));

            int cx = depth->cols / 2;
            int cy = depth->rows / 2;
            ushort dist = depth->at<ushort>(cy, cx);
            m_distanceIndicator->setValue(static_cast<int>(dist));
            emit distanceChanged(static_cast<int>(dist));
        }
    }

    m_totalFrames++;
    double elapsed = m_scanTimer.elapsed() / 1000.0;
    double fps = (elapsed > 0) ? (m_totalFrames / elapsed) : 0.0;
    m_fpsLabel->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
    m_frameCountLabel->setText(QString("Frames: %1").arg(m_totalFrames));
    int sec = static_cast<int>(elapsed);
    m_timeLabel->setText(QString("Time: %1:%2")
                         .arg(sec / 60, 2, 10, QChar('0'))
                         .arg(sec % 60, 2, 10, QChar('0')));
}

void MainWindow::onPointCloudReady(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (!cloud || cloud->empty() || !m_viewer || !m_cloudProcessing) return;

    if (m_scanning) {
        m_frameCounter++;
        if (m_frameCounter % m_frameSkip == 0) {
            QMutexLocker locker(&m_cloudMutex);
            *m_accumulatedCloud += *cloud;
            if (m_accumulatedCloud->size() > 200000) {
                pcl::VoxelGrid<pcl::PointXYZRGB> bigVoxel;
                bigVoxel.setInputCloud(m_accumulatedCloud);
                bigVoxel.setLeafSize(0.01f, 0.01f, 0.01f);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
                bigVoxel.filter(*temp);
                *m_accumulatedCloud = *temp;
            }
            emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
        }
    } else {
        m_viewer->removeAllPointClouds();
        m_viewer->addPointCloud(cloud, "live");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "live");
        m_vtkWidget->renderWindow()->Render();
    }
}

void MainWindow::updateViewer()
{
    if (!m_viewer) return;
    QMutexLocker locker(&m_cloudMutex);
    if (!m_accumulatedCloud || m_accumulatedCloud->empty()) return;

    if (m_scanning) {
        bool updated = m_viewer->updatePointCloud(m_accumulatedCloud, "accumulated");
        if (!updated) {
            m_viewer->addPointCloud(m_accumulatedCloud, "accumulated");
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "accumulated");
        }
        m_vtkWidget->renderWindow()->Render();
    }
}

void MainWindow::onCaptureError(const QString& msg)
{
    qCritical() << "Capture error:" << msg;
    statusBar()->showMessage("Error: " + msg, 5000);
    onStopClicked();
}

// ========== Калибровка ==========
void MainWindow::onCalibPreviewClicked()
{
    onPreviewClicked();
}

void MainWindow::onCalibCaptureClicked()
{
    if (m_lastColorFrame.empty()) {
        m_calibrator->statusChanged("Нет доступного RGB кадра. Запустите Preview.");
        return;
    }
    m_calibrator->addFrame(m_lastColorFrame);
}

void MainWindow::onCalibCalibrateClicked()
{
    if (m_calibrator->calibrate()) {
        m_calibrator->saveToFile("data/camera_calibration.xml");
        qInfo() << "Calibration saved";
    }
}

void MainWindow::onCalibResetClicked()
{
    m_calibrator->reset();
    m_calibFrameCountLabel->setText("Кадров: 0 / 12");
    m_calibCalibrateBtn->setEnabled(false);
}

void MainWindow::onCalibrationStatus(const QString &msg)
{
    m_calibStatusLabel->setText(msg);
}
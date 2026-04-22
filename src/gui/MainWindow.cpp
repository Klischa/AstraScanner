#include "MainWindow.h"
#include "../capture/CaptureWorker.h"
#include "../calibration/CameraCalibrator.h"
#include "LiveCloudWindow.h"
#include "SettingsDialog.h"
#include "LassoSelector.h"
#include "../filters/PointCloudFilters.h"
#include "../settings/SettingsManager.h"
#include "../export/ExportManager.h"
#include "../project/ProjectManager.h"

#include <vtkRenderer.h>
#include <vtkCamera.h>

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
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QDateTime>
#include <QDir>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <QDebug>
#include <filesystem>

std::atomic<MainWindow *> g_mainWindow{nullptr};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_accumulatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    g_mainWindow.store(this, std::memory_order_release);

    std::error_code ec;
    std::filesystem::create_directories("data", ec);

    m_liveCloudWindow = new LiveCloudWindow(this);
    m_filters = new PointCloudFilters(this);
    m_project = new ProjectManager(this);
    m_exporter = new ExportManager(this);
    connect(m_project, &ProjectManager::projectChanged, this, [this]() {
        if (m_projectStatusLabel) {
            const QString base = m_project->isOpen()
                                     ? QString("Проект: %1").arg(m_project->projectName())
                                     : QString("Проект: (не открыт)");
            m_projectStatusLabel->setText(m_project->isDirty() ? base + " *" : base);
        }
    });
    connect(m_project, &ProjectManager::scansChanged, this, &MainWindow::refreshScansList);

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
    g_mainWindow.store(nullptr, std::memory_order_release);
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

    // --- Меню «Файл» / «Экспорт» ---
    QMenu *fileMenu = menuBar()->addMenu("&Файл");
    QAction *newAct = fileMenu->addAction("Новый проект…");
    QAction *openAct = fileMenu->addAction("Открыть проект…");
    QAction *saveAct = fileMenu->addAction("Сохранить проект");
    QAction *saveAsAct = fileMenu->addAction("Сохранить проект как…");
    newAct->setShortcut(QKeySequence::New);
    openAct->setShortcut(QKeySequence::Open);
    saveAct->setShortcut(QKeySequence::Save);
    saveAsAct->setShortcut(QKeySequence::SaveAs);
    connect(newAct,    &QAction::triggered, this, &MainWindow::onNewProject);
    connect(openAct,   &QAction::triggered, this, &MainWindow::onOpenProject);
    connect(saveAct,   &QAction::triggered, this, &MainWindow::onSaveProject);
    connect(saveAsAct, &QAction::triggered, this, &MainWindow::onSaveProjectAs);

    QMenu *exportMenu = menuBar()->addMenu("&Экспорт");
    QAction *exportCloudAct = exportMenu->addAction("Экспорт текущего облака…");
    QAction *exportMeshAct  = exportMenu->addAction("Экспорт меша…");
    connect(exportCloudAct, &QAction::triggered, this, &MainWindow::onExportCurrentCloud);
    connect(exportMeshAct,  &QAction::triggered, this, &MainWindow::onExportMesh);

    QMenu *settingsMenu = menuBar()->addMenu("&Настройки");
    QAction *settingsAct = settingsMenu->addAction("Параметры…");
    settingsAct->setShortcut(QKeySequence("Ctrl+,"));
    connect(settingsAct, &QAction::triggered, this, &MainWindow::onShowSettingsDialog);

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

    QGroupBox* distanceGroup = new QGroupBox("Индикатор расстояния (активен на вкладке «Сканирование»)", this);
    QVBoxLayout* distanceLayout = new QVBoxLayout(distanceGroup);

    // m_distanceIndicator создаётся единожды во вкладке «Сканирование»,
    // чтобы один указатель всегда указывал на актуальный виджет. Здесь
    // выводим только подсказку.
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

    // --- Режим «поворотный стол» / авто-сохранение по таймеру ---
    // Пока включён и идёт сканирование, каждые N секунд текущее облако
    // сохраняется в проект как отдельный скан и m_accumulatedCloud
    // очищается. Между тиками пользователь вручную поворачивает объект
    // (или использует моторизованный стол). После того, как снято count
    // сканов, таймер останавливается автоматически. ICP-merge сканов
    // запускается отдельно на вкладке «Обработка».
    QGroupBox *turntableGroup = new QGroupBox(
        "Режим «поворотный стол» / авто-сохранение", this);
    QHBoxLayout *turntableLayout = new QHBoxLayout(turntableGroup);

    m_turntableEnableChk = new QCheckBox("Включить", this);
    m_turntableEnableChk->setToolTip(
        "Периодически сохраняет текущее облако как скан проекта и очищает "
        "накопитель. Работает только при открытом проекте и активном сканировании.");
    turntableLayout->addWidget(m_turntableEnableChk);

    turntableLayout->addWidget(new QLabel("Интервал:", this));
    m_turntableIntervalSpin = new QSpinBox(this);
    m_turntableIntervalSpin->setRange(1, 600);
    m_turntableIntervalSpin->setSuffix(" сек");
    m_turntableIntervalSpin->setValue(10);
    turntableLayout->addWidget(m_turntableIntervalSpin);

    turntableLayout->addWidget(new QLabel("Сканов:", this));
    m_turntableCountSpin = new QSpinBox(this);
    m_turntableCountSpin->setRange(1, 360);
    m_turntableCountSpin->setValue(12);
    m_turntableCountSpin->setToolTip("Остановить режим после стольких сохранённых сканов.");
    turntableLayout->addWidget(m_turntableCountSpin);

    m_turntableStatusLabel = new QLabel("Сохранено 0 сканов", this);
    turntableLayout->addWidget(m_turntableStatusLabel, 1);
    turntableLayout->addStretch();

    scanLayout->addWidget(turntableGroup);

    m_turntableTimer = new QTimer(this);
    connect(m_turntableTimer, &QTimer::timeout, this, &MainWindow::onTurntableTick);
    connect(m_turntableEnableChk, &QCheckBox::toggled,
            this, &MainWindow::onTurntableToggled);

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

    // --- Ручное редактирование облака (лассо) ---
    // Позволяет нарисовать произвольный контур во вьюере и удалить/оставить
    // только точки, проекция которых в экранных координатах попала в
    // полигон. Полезно для локальных выбросов, которые не хватает SOR/ROR —
    // например, стол, задний фон, блики. Перед каждой операцией делается
    // snapshot облака; «Отменить» возвращает последний.
    QGroupBox *lassoGroup = new QGroupBox("Ручное редактирование (лассо)", this);
    QVBoxLayout *lassoLayout = new QVBoxLayout(lassoGroup);

    QLabel *lassoHint = new QLabel(
        "Нарисуйте контур во вьюере левой кнопкой мыши. Esc — отменить.", this);
    lassoHint->setWordWrap(true);
    lassoLayout->addWidget(lassoHint);

    QHBoxLayout *lassoBtnRow = new QHBoxLayout();
    m_lassoDeleteBtn = new QPushButton("Удалить выделенное", this);
    m_lassoDeleteBtn->setToolTip(
        "Удалить из текущего облака все точки, попавшие в нарисованный контур.");
    m_lassoKeepBtn = new QPushButton("Оставить только выделенное", this);
    m_lassoKeepBtn->setToolTip(
        "Оставить только точки внутри контура; остальное — удалить.");
    m_undoEditBtn = new QPushButton("Отменить правку", this);
    m_undoEditBtn->setToolTip("Вернуть облако в состояние до последней правки.");
    m_undoEditBtn->setEnabled(false);

    lassoBtnRow->addWidget(m_lassoDeleteBtn);
    lassoBtnRow->addWidget(m_lassoKeepBtn);
    lassoBtnRow->addWidget(m_undoEditBtn);
    lassoBtnRow->addStretch();
    lassoLayout->addLayout(lassoBtnRow);

    m_lassoStatusLabel = new QLabel("Готов к редактированию", this);
    lassoLayout->addWidget(m_lassoStatusLabel);

    processingLayout->addWidget(lassoGroup);

    connect(m_lassoDeleteBtn, &QPushButton::clicked,
            this, &MainWindow::onLassoDeleteClicked);
    connect(m_lassoKeepBtn, &QPushButton::clicked,
            this, &MainWindow::onLassoKeepClicked);
    connect(m_undoEditBtn, &QPushButton::clicked,
            this, &MainWindow::onUndoEditClicked);

    QGroupBox* registrationGroup = new QGroupBox("Регистрация сканов (ICP)", this);
    QVBoxLayout* registrationLayout = new QVBoxLayout(registrationGroup);

    SettingsManager &settingsIcp = SettingsManager::instance();

    QHBoxLayout *icpParamsRow = new QHBoxLayout();
    QLabel *icpMaxCorrLabel = new QLabel("Max correspondence (м):", this);
    QDoubleSpinBox *icpMaxCorrSpin = new QDoubleSpinBox(this);
    icpMaxCorrSpin->setRange(0.001, 1.0);
    icpMaxCorrSpin->setDecimals(3);
    icpMaxCorrSpin->setSingleStep(0.005);
    icpMaxCorrSpin->setValue(settingsIcp.icpMaxCorrespondenceDistance());
    icpMaxCorrSpin->setToolTip("Максимальное расстояние, на котором ICP ищет пары точек. "
                               "Начните с 5 см; уменьшайте до 1–2 см после грубого выравнивания.");

    QLabel *icpIterLabel = new QLabel("Итерации:", this);
    QSpinBox *icpIterSpin = new QSpinBox(this);
    icpIterSpin->setRange(5, 500);
    icpIterSpin->setValue(settingsIcp.icpMaxIterations());

    QLabel *icpVoxelLabel = new QLabel("Финальный воксель (м):", this);
    QDoubleSpinBox *icpVoxelSpin = new QDoubleSpinBox(this);
    icpVoxelSpin->setRange(0.0, 0.05);
    icpVoxelSpin->setDecimals(4);
    icpVoxelSpin->setSingleStep(0.0005);
    icpVoxelSpin->setValue(settingsIcp.icpVoxelLeafOut());
    icpVoxelSpin->setToolTip("0 → без децимации. Полезно после мержа — N сканов "
                             "дают N×point density, а voxel этот дубль снимает.");

    QCheckBox *icpSkipCheck = new QCheckBox("Пропускать несошедшиеся", this);
    icpSkipCheck->setChecked(settingsIcp.icpSkipNonConverged());
    icpSkipCheck->setToolTip("Если ICP не сошёлся на скане — пропустить его, "
                             "а не добавлять «как есть». Безопаснее при плохих данных.");

    icpParamsRow->addWidget(icpMaxCorrLabel);
    icpParamsRow->addWidget(icpMaxCorrSpin);
    icpParamsRow->addSpacing(8);
    icpParamsRow->addWidget(icpIterLabel);
    icpParamsRow->addWidget(icpIterSpin);
    icpParamsRow->addSpacing(8);
    icpParamsRow->addWidget(icpVoxelLabel);
    icpParamsRow->addWidget(icpVoxelSpin);
    icpParamsRow->addStretch();
    registrationLayout->addLayout(icpParamsRow);
    registrationLayout->addWidget(icpSkipCheck);

    QHBoxLayout *mergeBtnRow = new QHBoxLayout();
    QPushButton *mergeScansBtn = new QPushButton("Объединить все сканы проекта", this);
    mergeScansBtn->setToolTip("Последовательная ICP-регистрация: scans[0] как опорный, "
                              "каждый следующий выравнивается относительно накопленного результата.");
    QPushButton *addMergedBtn = new QPushButton("Сохранить результат в проект…", this);
    addMergedBtn->setEnabled(false);
    mergeBtnRow->addWidget(mergeScansBtn);
    mergeBtnRow->addWidget(addMergedBtn);
    mergeBtnRow->addStretch();
    registrationLayout->addLayout(mergeBtnRow);

    QProgressBar* registrationProgress = new QProgressBar(this);
    registrationProgress->setRange(0, 100);
    registrationProgress->setValue(0);
    registrationLayout->addWidget(registrationProgress);

    QLabel *icpStatusLabel = new QLabel("", this);
    registrationLayout->addWidget(icpStatusLabel);

    processingLayout->addWidget(registrationGroup);
    m_mergeBtn = mergeScansBtn;
    m_addMergedBtn = addMergedBtn;
    m_icpStatusLabel = icpStatusLabel;

    // --- Poisson Surface Reconstruction ---
    QGroupBox *meshGroup = new QGroupBox("Реконструкция поверхности (Poisson)", this);
    QVBoxLayout *meshLayout = new QVBoxLayout(meshGroup);

    SettingsManager &settings = SettingsManager::instance();

    QHBoxLayout *poissonParamsRow = new QHBoxLayout();
    QLabel *depthLabel = new QLabel("Depth:", this);
    QSpinBox *depthSpin = new QSpinBox(this);
    depthSpin->setRange(5, 12);
    depthSpin->setValue(settings.poissonDepth());
    depthSpin->setToolTip("Глубина октодерева. 8≈256^3, 9≈512^3, 10≈1024^3. "
                          "Больше = детальнее, но квадратично больше памяти и времени.");

    QLabel *pointWeightLabel = new QLabel("Point weight:", this);
    QDoubleSpinBox *pointWeightSpin = new QDoubleSpinBox(this);
    pointWeightSpin->setRange(0.0, 20.0);
    pointWeightSpin->setSingleStep(0.5);
    pointWeightSpin->setValue(settings.poissonPointWeight());
    pointWeightSpin->setToolTip("Screened-Poisson weight. 0 = классический Poisson, "
                                "≥4 = screened (плотнее следует точкам).");

    QLabel *samplesLabel = new QLabel("Samples/node:", this);
    QDoubleSpinBox *samplesSpin = new QDoubleSpinBox(this);
    samplesSpin->setRange(1.0, 20.0);
    samplesSpin->setSingleStep(0.5);
    samplesSpin->setValue(settings.poissonSamplesPerNode());
    samplesSpin->setToolTip("Сколько точек должно попасть в лист октодерева. "
                            "Больше = меньше шумных нод.");

    poissonParamsRow->addWidget(depthLabel);
    poissonParamsRow->addWidget(depthSpin);
    poissonParamsRow->addSpacing(12);
    poissonParamsRow->addWidget(pointWeightLabel);
    poissonParamsRow->addWidget(pointWeightSpin);
    poissonParamsRow->addSpacing(12);
    poissonParamsRow->addWidget(samplesLabel);
    poissonParamsRow->addWidget(samplesSpin);
    poissonParamsRow->addStretch();
    meshLayout->addLayout(poissonParamsRow);

    QHBoxLayout *normalRow = new QHBoxLayout();
    QLabel *normalRadiusLabel = new QLabel("Normal radius (м):", this);
    QDoubleSpinBox *normalRadiusSpin = new QDoubleSpinBox(this);
    normalRadiusSpin->setRange(0.0, 0.1);
    normalRadiusSpin->setDecimals(4);
    normalRadiusSpin->setSingleStep(0.001);
    normalRadiusSpin->setValue(settings.poissonNormalRadius());
    normalRadiusSpin->setToolTip("Радиус поиска соседей для оценки нормалей. "
                                 "0 → использовать k ближайших (см. поле k).");

    QLabel *kNearestLabel = new QLabel("k ближайших:", this);
    QSpinBox *kNearestSpin = new QSpinBox(this);
    kNearestSpin->setRange(5, 100);
    kNearestSpin->setValue(settings.poissonKNearest());

    normalRow->addWidget(normalRadiusLabel);
    normalRow->addWidget(normalRadiusSpin);
    normalRow->addSpacing(12);
    normalRow->addWidget(kNearestLabel);
    normalRow->addWidget(kNearestSpin);
    normalRow->addStretch();
    meshLayout->addLayout(normalRow);

    // --- Ручная переориентация нормалей ---
    // По умолчанию computeNormals в PointCloudFilters задаёт view-point по
    // эвристике (центр облака, сдвинутый на 1 м «к камере» по Z). Для
    // сильно замкнутых объектов эта эвристика иногда выдаёт нормали наружу
    // на одной половине и внутрь на другой — Poisson в таком случае рисует
    // меш «наизнанку». Эти контролы позволяют задать ориентацию явно.
    QGroupBox *normalOrientGroup = new QGroupBox(
        "Ориентация нормалей (при проблемах с Poisson)", this);
    QVBoxLayout *normalOrientLayout = new QVBoxLayout(normalOrientGroup);

    m_poissonFlipNormalsChk = new QCheckBox("Инвертировать нормали (flip)", this);
    m_poissonFlipNormalsChk->setToolTip(
        "Развернуть все нормали на 180°. Помогает, если меш получился "
        "«вывернут наизнанку».");
    normalOrientLayout->addWidget(m_poissonFlipNormalsChk);

    m_poissonConsistentOrientChk = new QCheckBox(
        "Согласованная ориентация (BFS по k-соседям)", this);
    m_poissonConsistentOrientChk->setToolTip(
        "Распространяет ориентацию от seed-точки по ближайшим соседям так, "
        "чтобы нормали были сонаправлены. Помогает для замкнутых объектов, "
        "где единый view-point даёт локально противоположные нормали. "
        "Медленнее на больших облаках.");
    normalOrientLayout->addWidget(m_poissonConsistentOrientChk);

    QHBoxLayout *vpRow = new QHBoxLayout();
    m_poissonCustomVpChk = new QCheckBox("Custom view-point:", this);
    m_poissonCustomVpChk->setToolTip(
        "Использовать заданные координаты (X, Y, Z) в метрах как view-point "
        "для ориентации нормалей вместо эвристики (centroid - 1 м по Z).");
    vpRow->addWidget(m_poissonCustomVpChk);

    auto *vpXLabel = new QLabel("X:", this);
    m_poissonVpX = new QDoubleSpinBox(this);
    m_poissonVpX->setRange(-10.0, 10.0);
    m_poissonVpX->setDecimals(3);
    m_poissonVpX->setSingleStep(0.05);
    m_poissonVpX->setSuffix(" м");
    m_poissonVpX->setEnabled(false);

    auto *vpYLabel = new QLabel("Y:", this);
    m_poissonVpY = new QDoubleSpinBox(this);
    m_poissonVpY->setRange(-10.0, 10.0);
    m_poissonVpY->setDecimals(3);
    m_poissonVpY->setSingleStep(0.05);
    m_poissonVpY->setSuffix(" м");
    m_poissonVpY->setEnabled(false);

    auto *vpZLabel = new QLabel("Z:", this);
    m_poissonVpZ = new QDoubleSpinBox(this);
    m_poissonVpZ->setRange(-10.0, 10.0);
    m_poissonVpZ->setDecimals(3);
    m_poissonVpZ->setSingleStep(0.05);
    m_poissonVpZ->setSuffix(" м");
    m_poissonVpZ->setValue(-1.0);   // эквивалент дефолтной эвристики
    m_poissonVpZ->setEnabled(false);

    vpRow->addWidget(vpXLabel);
    vpRow->addWidget(m_poissonVpX);
    vpRow->addWidget(vpYLabel);
    vpRow->addWidget(m_poissonVpY);
    vpRow->addWidget(vpZLabel);
    vpRow->addWidget(m_poissonVpZ);
    vpRow->addStretch();
    normalOrientLayout->addLayout(vpRow);

    connect(m_poissonCustomVpChk, &QCheckBox::toggled, this,
            [this](bool on) {
                m_poissonVpX->setEnabled(on);
                m_poissonVpY->setEnabled(on);
                m_poissonVpZ->setEnabled(on);
            });

    meshLayout->addWidget(normalOrientGroup);

    QPushButton *reconstructBtn = new QPushButton("Построить меш", this);
    QPushButton *showCloudBtn   = new QPushButton("Показать облако", this);
    QPushButton *exportMeshBtnP = new QPushButton("Экспорт меша…", this);
    QHBoxLayout *meshBtnRow = new QHBoxLayout();
    meshBtnRow->addWidget(reconstructBtn);
    meshBtnRow->addWidget(showCloudBtn);
    meshBtnRow->addWidget(exportMeshBtnP);
    meshBtnRow->addStretch();
    meshLayout->addLayout(meshBtnRow);

    m_poissonProgress = new QProgressBar(this);
    m_poissonProgress->setRange(0, 100);
    m_poissonProgress->setValue(0);
    meshLayout->addWidget(m_poissonProgress);

    m_meshStatusLabel = new QLabel("Меш не построен", this);
    meshLayout->addWidget(m_meshStatusLabel);

    processingLayout->addWidget(meshGroup);

    // Прогресс Poisson — через `progressUpdated` от PointCloudFilters. Он
    // эмитится и при ICP-мерже, чтобы не было путаницы между индикаторами,
    // сбрасываем значение при начале реконструкции.
    connect(m_filters, &PointCloudFilters::progressUpdated,
            m_poissonProgress, &QProgressBar::setValue);

    connect(reconstructBtn, &QPushButton::clicked, this,
        [this, depthSpin, pointWeightSpin, samplesSpin, normalRadiusSpin, kNearestSpin, reconstructBtn]() {
            PointCloudFilters::PoissonParams p;
            p.depth = depthSpin->value();
            p.pointWeight = static_cast<float>(pointWeightSpin->value());
            p.samplesPerNode = static_cast<float>(samplesSpin->value());
            p.normalSearchRadius = normalRadiusSpin->value();
            p.kNearest = kNearestSpin->value();

            // Ручная переориентация нормалей, если пользователь включил.
            p.flipNormals = m_poissonFlipNormalsChk && m_poissonFlipNormalsChk->isChecked();
            p.consistentOrientation = m_poissonConsistentOrientChk && m_poissonConsistentOrientChk->isChecked();
            if (m_poissonCustomVpChk && m_poissonCustomVpChk->isChecked()) {
                p.useCustomViewpoint = true;
                p.viewpointX = static_cast<float>(m_poissonVpX->value());
                p.viewpointY = static_cast<float>(m_poissonVpY->value());
                p.viewpointZ = static_cast<float>(m_poissonVpZ->value());
            }

            SettingsManager &s = SettingsManager::instance();
            s.setPoissonDepth(p.depth);
            s.setPoissonPointWeight(p.pointWeight);
            s.setPoissonSamplesPerNode(p.samplesPerNode);
            s.setPoissonNormalRadius(p.normalSearchRadius);
            s.setPoissonKNearest(p.kNearest);

            reconstructBtn->setEnabled(false);
            onReconstructMeshClicked(p);
        });

    connect(showCloudBtn, &QPushButton::clicked, this, &MainWindow::onShowCloudClicked);
    connect(exportMeshBtnP, &QPushButton::clicked, this, &MainWindow::onExportMesh);

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

    connect(mergeScansBtn, &QPushButton::clicked, this,
        [this, icpMaxCorrSpin, icpIterSpin, icpVoxelSpin, icpSkipCheck]() {
            PointCloudFilters::MergeParams p;
            p.maxCorrespondenceDistance = icpMaxCorrSpin->value();
            p.maximumIterations = icpIterSpin->value();
            p.voxelLeafOut = icpVoxelSpin->value();
            p.skipNonConverged = icpSkipCheck->isChecked();

            SettingsManager &s = SettingsManager::instance();
            s.setIcpMaxCorrespondenceDistance(p.maxCorrespondenceDistance);
            s.setIcpMaxIterations(p.maximumIterations);
            s.setIcpVoxelLeafOut(p.voxelLeafOut);
            s.setIcpSkipNonConverged(p.skipNonConverged);

            onMergeScansClicked(p);
        });

    connect(addMergedBtn, &QPushButton::clicked, this, &MainWindow::onSaveMergedToProject);

    connect(m_filters, &PointCloudFilters::progressUpdated, registrationProgress, &QProgressBar::setValue);
    connect(m_filters, &PointCloudFilters::filterCompleted, this, [this](const QString &filter, int before, int after) {
        statusBar()->showMessage(QString("%1: %2 -> %3 точек").arg(filter).arg(before).arg(after), 3000);
    });

    // Вкладка "Проект" — список сохранённых сканов + кнопки управления.
    QWidget *projectTab = new QWidget();
    m_tabWidget->addTab(projectTab, "Проект");
    QVBoxLayout *projectLayout = new QVBoxLayout(projectTab);

    m_projectStatusLabel = new QLabel("Проект: (не открыт)", this);
    projectLayout->addWidget(m_projectStatusLabel);

    QHBoxLayout *projectBtnRow = new QHBoxLayout();
    QPushButton *addScanBtn = new QPushButton("Добавить текущее облако как скан", this);
    QPushButton *exportCloudBtn = new QPushButton("Экспорт текущего облака…", this);
    QPushButton *exportMeshBtn = new QPushButton("Экспорт меша…", this);
    projectBtnRow->addWidget(addScanBtn);
    projectBtnRow->addWidget(exportCloudBtn);
    projectBtnRow->addWidget(exportMeshBtn);
    projectBtnRow->addStretch();
    projectLayout->addLayout(projectBtnRow);

    m_scansList = new QListWidget(this);
    m_scansList->setContextMenuPolicy(Qt::CustomContextMenu);
    projectLayout->addWidget(m_scansList, 1);

    connect(addScanBtn, &QPushButton::clicked, this, &MainWindow::onAddCurrentCloudToProject);
    connect(exportCloudBtn, &QPushButton::clicked, this, &MainWindow::onExportCurrentCloud);
    connect(exportMeshBtn, &QPushButton::clicked, this, &MainWindow::onExportMesh);
    connect(m_scansList, &QListWidget::customContextMenuRequested,
            this, &MainWindow::onScansListContextMenu);
    connect(m_scansList, &QListWidget::itemDoubleClicked,
            this, &MainWindow::onScansListDoubleClicked);

    // Вкладка "Логи" — fileMessageHandler форвардит сюда все сообщения Qt.
    QWidget* logTab = new QWidget();
    m_tabWidget->addTab(logTab, "Логи");
    QVBoxLayout* logLayout = new QVBoxLayout(logTab);
    m_logTextEdit = new QTextEdit(this);
    m_logTextEdit->setReadOnly(true);
    m_logTextEdit->setLineWrapMode(QTextEdit::NoWrap);
    m_logTextEdit->setFontFamily("monospace");
    logLayout->addWidget(m_logTextEdit);
    QPushButton* clearLogBtn = new QPushButton("Очистить", this);
    clearLogBtn->setMaximumWidth(120);
    connect(clearLogBtn, &QPushButton::clicked, m_logTextEdit, &QTextEdit::clear);
    logLayout->addWidget(clearLogBtn);

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

    // Сохраняем renderer под рукой — понадобится LassoSelector для
    // проецирования точек облака в display-координаты.
    m_vtkRenderer = renderer;
    m_lasso = new LassoSelector(m_vtkWidget, m_vtkRenderer, this);
    connect(m_lasso, &LassoSelector::lassoCompleted,
            this, &MainWindow::onLassoCompleted);
    connect(m_lasso, &LassoSelector::lassoCanceled,
            this, &MainWindow::onLassoCanceled);

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
    // На паузе отключаем обработку облаков, чтобы воркер не занимался лишней
    // работой (voxelization/построение облака) пока сканирование приостановлено.
    m_cloudProcessing = m_scanning;
    if (m_worker) {
        m_worker->setCloudProcessingEnabled(m_cloudProcessing);
    }
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
    connect(m_worker, &CaptureWorker::warning, this, &MainWindow::onCaptureWarning);
    connect(m_worker, &CaptureWorker::frameProcessed, this, &MainWindow::onFrameProcessed);
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
    }
    if (m_captureThread) {
        m_captureThread->quit();
        // Даём воркеру достаточно времени корректно завершиться. process()
        // проверяет m_running на каждой итерации, поэтому terminate() здесь
        // не нужен и опасен (может оставить OpenNI/OpenCV в несогласованном
        // состоянии).
        if (!m_captureThread->wait(10000)) {
            qWarning() << "Capture thread did not stop within 10s";
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
    if (!guiTimer.isValid()) guiTimer.start();

    // Обновляем GUI не чаще 10 раз в секунду
    bool updateGui = (guiTimer.elapsed() > 100);
    if (updateGui) {
        guiTimer.restart();
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

    // Счётчик кадров и FPS обновляется в onFrameProcessed() — он приходит на
    // каждый кадр, а onNewFrame дросселируется до 10 Гц для обновления GUI.
    double elapsed = m_scanTimer.elapsed() / 1000.0;
    int sec = static_cast<int>(elapsed);
    m_timeLabel->setText(QString("Time: %1:%2")
                         .arg(sec / 60, 2, 10, QChar('0'))
                         .arg(sec % 60, 2, 10, QChar('0')));
}

void MainWindow::onFrameProcessed(int totalFrames)
{
    m_totalFrames = totalFrames;
    double elapsed = m_scanTimer.elapsed() / 1000.0;
    double fps = (elapsed > 0) ? (m_totalFrames / elapsed) : 0.0;
    m_fpsLabel->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
    m_frameCountLabel->setText(QString("Frames: %1").arg(m_totalFrames));
}

void MainWindow::onCaptureWarning(const QString &msg)
{
    qWarning() << "[Capture warning]" << msg;
    statusBar()->showMessage(msg, 5000);
    if (m_logTextEdit) {
        appendLog(QString("[WARN] %1").arg(msg));
    }
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
        m_calibrator->notifyStatus("Нет доступного RGB кадра. Запустите Preview.");
        return;
    }
    m_calibrator->addFrame(m_lastColorFrame);
}

void MainWindow::onCalibCalibrateClicked()
{
    if (m_calibrator->calibrate()) {
        const double rms = m_calibrator->reprojectionError();
        if (rms > 2.0) {
            qWarning() << "Calibration RMS is high:" << rms << "px. Result may be inaccurate.";
            m_calibrator->notifyStatus(
                QString("Калибровка выполнена с большой ошибкой (RMS=%1). "
                        "Рекомендуется повторить.").arg(rms, 0, 'f', 3));
        }
        m_calibrator->saveToFile("data/camera_calibration.xml");
        qInfo() << "Calibration saved, RMS:" << rms;
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

// ========== Проект ==========

void MainWindow::onNewProject()
{
    if (m_project->isDirty()) {
        auto r = QMessageBox::question(this, "Новый проект",
            "Текущий проект не сохранён. Создать новый и отбросить изменения?");
        if (r != QMessageBox::Yes) return;
    }

    const QString baseDir = SettingsManager::instance().projectsDirectory();
    const QString dir = QFileDialog::getSaveFileName(
        this, "Создать новый проект (укажите папку)", baseDir,
        QString(), nullptr, QFileDialog::ShowDirsOnly | QFileDialog::DontConfirmOverwrite);
    if (dir.isEmpty()) return;

    if (!m_project->newProject(dir)) {
        QMessageBox::warning(this, "Ошибка", m_project->lastError());
        return;
    }
    SettingsManager::instance().setProjectsDirectory(QFileInfo(dir).absolutePath());
    refreshScansList();
}

void MainWindow::onOpenProject()
{
    if (m_project->isDirty()) {
        auto r = QMessageBox::question(this, "Открыть проект",
            "Текущий проект не сохранён. Открыть другой и отбросить изменения?");
        if (r != QMessageBox::Yes) return;
    }
    const QString baseDir = SettingsManager::instance().projectsDirectory();
    const QString dir = QFileDialog::getExistingDirectory(
        this, "Выберите папку проекта", baseDir);
    if (dir.isEmpty()) return;

    if (!m_project->openProject(dir)) {
        QMessageBox::warning(this, "Ошибка", m_project->lastError());
        return;
    }
    SettingsManager::instance().setProjectsDirectory(QFileInfo(dir).absolutePath());
    refreshScansList();
}

void MainWindow::onSaveProject()
{
    if (!m_project->isOpen()) {
        onSaveProjectAs();
        return;
    }
    if (!m_project->saveProject()) {
        QMessageBox::warning(this, "Ошибка", m_project->lastError());
    } else {
        statusBar()->showMessage("Проект сохранён", 3000);
    }
}

void MainWindow::onSaveProjectAs()
{
    const QString baseDir = SettingsManager::instance().projectsDirectory();
    const QString dir = QFileDialog::getSaveFileName(
        this, "Сохранить проект как…", baseDir,
        QString(), nullptr, QFileDialog::ShowDirsOnly | QFileDialog::DontConfirmOverwrite);
    if (dir.isEmpty()) return;

    if (!m_project->isOpen()) {
        if (!m_project->newProject(dir)) {
            QMessageBox::warning(this, "Ошибка", m_project->lastError());
            return;
        }
    } else if (!m_project->saveProjectAs(dir)) {
        QMessageBox::warning(this, "Ошибка", m_project->lastError());
        return;
    }
    SettingsManager::instance().setProjectsDirectory(QFileInfo(dir).absolutePath());
    refreshScansList();
}

void MainWindow::onAddCurrentCloudToProject()
{
    QMutexLocker locker(&m_cloudMutex);
    if (!m_accumulatedCloud || m_accumulatedCloud->empty()) {
        QMessageBox::information(this, "Проект", "Текущее облако пустое.");
        return;
    }
    if (!m_project->isOpen()) {
        auto r = QMessageBox::question(this, "Проект не открыт",
            "Проект не открыт. Создать новый проект?");
        if (r != QMessageBox::Yes) return;
        locker.unlock();
        onNewProject();
        locker.relock();
        if (!m_project->isOpen()) return;
    }

    // Копируем облако, чтобы дальнейшие изменения m_accumulatedCloud не
    // влияли на сохранённый скан.
    auto snapshot = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*m_accumulatedCloud);
    bool ok = false;
    const QString name = QInputDialog::getText(this, "Имя скана",
        "Введите имя скана:", QLineEdit::Normal,
        QString("Scan %1").arg(m_project->scanCount() + 1), &ok);
    if (!ok) return;

    // addScan использует файловый I/O — временно отпускаем мьютекс облака,
    // чтобы не блокировать пайплайн захвата.
    locker.unlock();
    const int idx = m_project->addScan(snapshot, name);
    if (idx < 0) {
        QMessageBox::warning(this, "Ошибка", m_project->lastError());
        return;
    }
    statusBar()->showMessage(QString("Скан добавлен в проект (%1 точек)")
                                 .arg(snapshot->size()),
                             3000);
}

void MainWindow::onExportCurrentCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy;
    {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) {
            QMessageBox::information(this, "Экспорт", "Текущее облако пустое.");
            return;
        }
        cloudCopy = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*m_accumulatedCloud);
    }

    const QString baseDir = SettingsManager::instance().lastExportDirectory();
    const QString filter = ExportManager::cloudFileFilters().join(";;");
    QString selected;
    const QString filename = QFileDialog::getSaveFileName(
        this, "Экспорт облака", baseDir, filter, &selected);
    if (filename.isEmpty()) return;

    SettingsManager::instance().setLastExportDirectory(QFileInfo(filename).absolutePath());
    if (!m_exporter->savePointCloud(cloudCopy, filename)) {
        QMessageBox::warning(this, "Ошибка экспорта", m_exporter->lastError());
        return;
    }
    statusBar()->showMessage(QString("Экспортировано: %1").arg(filename), 5000);
}

void MainWindow::onExportMesh()
{
    if (m_lastMesh.polygons.empty()) {
        QMessageBox::information(this, "Экспорт меша",
            "Меш ещё не построен. Сначала выполните реконструкцию поверхности.");
        return;
    }

    const QString baseDir = SettingsManager::instance().lastExportDirectory();
    const QString filter = ExportManager::meshFileFilters().join(";;");
    QString selected;
    const QString filename = QFileDialog::getSaveFileName(
        this, "Экспорт меша", baseDir, filter, &selected);
    if (filename.isEmpty()) return;

    SettingsManager::instance().setLastExportDirectory(QFileInfo(filename).absolutePath());
    if (!m_exporter->savePolygonMesh(m_lastMesh, filename)) {
        QMessageBox::warning(this, "Ошибка экспорта", m_exporter->lastError());
        return;
    }
    statusBar()->showMessage(QString("Меш экспортирован: %1").arg(filename), 5000);
}

void MainWindow::onScansListContextMenu(const QPoint &pos)
{
    QListWidgetItem *item = m_scansList->itemAt(pos);
    if (!item) return;
    const int idx = m_scansList->row(item);

    QMenu menu(this);
    QAction *loadAct   = menu.addAction("Загрузить как текущее облако");
    QAction *renameAct = menu.addAction("Переименовать…");
    QAction *exportAct = menu.addAction("Экспорт…");
    menu.addSeparator();
    QAction *removeAct = menu.addAction("Удалить");

    QAction *chosen = menu.exec(m_scansList->mapToGlobal(pos));
    if (!chosen) return;
    if (chosen == loadAct) {
        auto cloud = m_project->scanCloud(idx);
        if (!cloud) {
            QMessageBox::warning(this, "Ошибка", m_project->lastError());
            return;
        }
        QMutexLocker locker(&m_cloudMutex);
        *m_accumulatedCloud = *cloud;
        emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
        updateViewer();
    } else if (chosen == renameAct) {
        bool ok = false;
        const QString newName = QInputDialog::getText(this, "Переименовать скан",
            "Новое имя:", QLineEdit::Normal,
            m_project->scans().at(idx).name, &ok);
        if (ok && !newName.isEmpty()) {
            m_project->renameScan(idx, newName);
        }
    } else if (chosen == exportAct) {
        auto cloud = m_project->scanCloud(idx);
        if (!cloud) {
            QMessageBox::warning(this, "Ошибка", m_project->lastError());
            return;
        }
        const QString baseDir = SettingsManager::instance().lastExportDirectory();
        const QString filter = ExportManager::cloudFileFilters().join(";;");
        const QString filename = QFileDialog::getSaveFileName(
            this, "Экспорт скана", baseDir, filter);
        if (filename.isEmpty()) return;
        SettingsManager::instance().setLastExportDirectory(QFileInfo(filename).absolutePath());
        if (!m_exporter->savePointCloud(cloud, filename)) {
            QMessageBox::warning(this, "Ошибка", m_exporter->lastError());
        }
    } else if (chosen == removeAct) {
        auto r = QMessageBox::question(this, "Удалить скан",
            QString("Удалить скан «%1»?").arg(m_project->scans().at(idx).name));
        if (r == QMessageBox::Yes) m_project->removeScan(idx);
    }
}

void MainWindow::onScansListDoubleClicked(QListWidgetItem *item)
{
    if (!item) return;
    const int idx = m_scansList->row(item);
    auto cloud = m_project->scanCloud(idx);
    if (!cloud) {
        QMessageBox::warning(this, "Ошибка", m_project->lastError());
        return;
    }
    QMutexLocker locker(&m_cloudMutex);
    *m_accumulatedCloud = *cloud;
    emit cloudSizeChanged(static_cast<int>(m_accumulatedCloud->size()));
    updateViewer();
}

void MainWindow::refreshScansList()
{
    if (!m_scansList) return;
    m_scansList->clear();
    for (const ScanItem &item : m_project->scans()) {
        const QString label = QString("%1  (%2 точек, %3)")
                                  .arg(item.name)
                                  .arg(item.pointCount)
                                  .arg(item.createdAt.toString("yyyy-MM-dd HH:mm"));
        m_scansList->addItem(label);
    }
    if (m_projectStatusLabel) {
        const QString base = m_project->isOpen()
                                 ? QString("Проект: %1").arg(m_project->projectName())
                                 : QString("Проект: (не открыт)");
        m_projectStatusLabel->setText(m_project->isDirty() ? base + " *" : base);
    }
}

// ========== Poisson-реконструкция ==========

void MainWindow::onReconstructMeshClicked(const PointCloudFilters::PoissonParams &params)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr snapshot;
    {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) {
            QMessageBox::information(this, "Реконструкция", "Облако пустое.");
            return;
        }
        // Копия, чтобы не держать mutex пока Poisson работает секунды-минуты.
        snapshot = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*m_accumulatedCloud);
    }

    if (m_poissonWatcher && m_poissonWatcher->isRunning()) {
        QMessageBox::information(this, "Реконструкция",
            "Реконструкция уже запущена — дождитесь её завершения.");
        return;
    }

    if (m_meshStatusLabel) m_meshStatusLabel->setText(
        QString("Выполняется реконструкция… (%1 точек, depth=%2)")
            .arg(snapshot->size()).arg(params.depth));
    if (m_poissonProgress) m_poissonProgress->setValue(0);
    statusBar()->showMessage("Poisson: идёт реконструкция…");

    // Не создаём m_filters заново — он qobject с сигналом progressUpdated,
    // который уже подключён к m_poissonProgress. Но сам вызов выполняется в
    // worker-потоке через QtConcurrent::run.
    if (!m_poissonWatcher) {
        m_poissonWatcher = new QFutureWatcher<pcl::PolygonMesh>(this);
        connect(m_poissonWatcher, &QFutureWatcher<pcl::PolygonMesh>::finished,
                this, &MainWindow::onPoissonFinished);
    }

    QFuture<pcl::PolygonMesh> future = QtConcurrent::run([this, snapshot, params]() {
        return m_filters->reconstructPoissonMesh(snapshot, params);
    });
    m_poissonWatcher->setFuture(future);
}

void MainWindow::onPoissonFinished()
{
    if (!m_poissonWatcher) return;
    const pcl::PolygonMesh mesh = m_poissonWatcher->result();

    // Повторно активируем кнопку «Построить меш» — ищем её по группе меша.
    for (QPushButton *btn : findChildren<QPushButton*>()) {
        if (btn->text() == "Построить меш") btn->setEnabled(true);
    }

    if (mesh.polygons.empty()) {
        statusBar()->showMessage("Poisson: реконструкция не удалась", 5000);
        if (m_meshStatusLabel) m_meshStatusLabel->setText("Меш не построен (см. лог)");
        if (m_poissonProgress) m_poissonProgress->setValue(0);
        QMessageBox::warning(this, "Реконструкция",
            "Poisson не смог построить меш. Возможные причины: "
            "слишком редкое облако, неверно ориентированные нормали, "
            "нехватка памяти. Подробности — на вкладке «Логи».");
        return;
    }

    m_lastMesh = mesh;
    const qulonglong nPoly = static_cast<qulonglong>(mesh.polygons.size());
    if (m_meshStatusLabel) m_meshStatusLabel->setText(
        QString("Меш построен: %1 полигонов").arg(nPoly));
    statusBar()->showMessage(QString("Poisson: готов меш из %1 полигонов").arg(nPoly), 5000);
    if (m_poissonProgress) m_poissonProgress->setValue(100);

    // Показываем меш в вьюаре. Убираем накопленное облако, чтобы не
    // перекрывало меш — пользователь может вернуть его кнопкой «Показать
    // облако».
    if (m_viewer) {
        m_viewer->removeAllPointClouds();
        if (!m_meshViewerId.isEmpty()) {
            m_viewer->removePolygonMesh(m_meshViewerId.toStdString());
        }
        m_meshViewerId = "poisson_mesh";
        m_viewer->addPolygonMesh(m_lastMesh, m_meshViewerId.toStdString());
        m_vtkWidget->renderWindow()->Render();
    }
}

// ========== ICP-регистрация (merge проекта) ==========

void MainWindow::onMergeScansClicked(const PointCloudFilters::MergeParams &params)
{
    if (!m_project || !m_project->isOpen() || m_project->scanCount() < 2) {
        QMessageBox::information(this, "Объединение",
            "Для объединения нужен открытый проект с ≥ 2 сканами. "
            "Добавьте сканы через вкладку «Проект».");
        return;
    }

    if (m_mergeWatcher && m_mergeWatcher->isRunning()) {
        QMessageBox::information(this, "Объединение",
            "Объединение уже запущено — дождитесь завершения.");
        return;
    }

    // Подгружаем все облака синхронно (disk I/O — быстро, в отличие от ICP).
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> scans;
    scans.reserve(m_project->scanCount());
    for (int i = 0; i < m_project->scanCount(); ++i) {
        auto c = m_project->scanCloud(i);
        if (!c || c->empty()) {
            qWarning() << "[merge] skipping empty scan" << i;
            continue;
        }
        scans.push_back(c);
    }
    if (scans.size() < 2) {
        QMessageBox::information(this, "Объединение",
            "После загрузки осталось < 2 непустых сканов.");
        return;
    }

    if (m_mergeBtn) m_mergeBtn->setEnabled(false);
    if (m_addMergedBtn) m_addMergedBtn->setEnabled(false);
    if (m_icpStatusLabel) m_icpStatusLabel->setText(
        QString("Идёт ICP-регистрация %1 сканов…").arg(scans.size()));
    statusBar()->showMessage("ICP: объединение сканов…");

    if (!m_mergeWatcher) {
        m_mergeWatcher = new QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(this);
        connect(m_mergeWatcher,
                &QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::finished,
                this, &MainWindow::onMergeFinished);
    }

    QFuture<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> future =
        QtConcurrent::run([this, scans, params]() {
            return m_filters->mergeScans(scans, params);
        });
    m_mergeWatcher->setFuture(future);
}

void MainWindow::onMergeFinished()
{
    if (!m_mergeWatcher) return;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged = m_mergeWatcher->result();

    if (m_mergeBtn) m_mergeBtn->setEnabled(true);

    if (!merged || merged->empty()) {
        if (m_icpStatusLabel) m_icpStatusLabel->setText("Объединение не удалось (см. лог)");
        statusBar()->showMessage("ICP: пустой результат", 5000);
        QMessageBox::warning(this, "Объединение",
            "ICP вернул пустое облако. Возможно, все сканы были пустыми "
            "или не сошлись.");
        return;
    }

    m_lastMerged = merged;
    const int npts = static_cast<int>(merged->size());
    if (m_icpStatusLabel) m_icpStatusLabel->setText(
        QString("Готово: %1 точек в объединённом облаке").arg(npts));
    statusBar()->showMessage(
        QString("ICP: объединено в %1 точек").arg(npts), 5000);
    if (m_addMergedBtn) m_addMergedBtn->setEnabled(true);

    // Показываем результат как текущее накопленное облако.
    {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud) {
            m_accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        }
        *m_accumulatedCloud = *merged;
    }
    emit cloudSizeChanged(npts);

    // Если в вьюаре был меш — убираем, чтобы облако было видно.
    if (m_viewer && !m_meshViewerId.isEmpty()) {
        m_viewer->removePolygonMesh(m_meshViewerId.toStdString());
        m_meshViewerId.clear();
    }
    updateViewer();
}

void MainWindow::onSaveMergedToProject()
{
    if (!m_lastMerged || m_lastMerged->empty()) {
        QMessageBox::information(this, "Сохранение",
            "Нет объединённого облака. Сначала выполните «Объединить все сканы проекта».");
        return;
    }
    if (!m_project || !m_project->isOpen()) {
        QMessageBox::warning(this, "Сохранение", "Проект не открыт.");
        return;
    }

    bool ok = false;
    const QString defaultName =
        QString("merged_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    const QString name = QInputDialog::getText(this, "Новый скан",
        "Имя:", QLineEdit::Normal, defaultName, &ok);
    if (!ok || name.trimmed().isEmpty()) return;

    const int index = m_project->addScan(m_lastMerged, name.trimmed());
    if (index < 0) {
        QMessageBox::critical(this, "Сохранение",
            QString("Не удалось добавить скан:\n%1").arg(m_project->lastError()));
        return;
    }
    refreshScansList();
    statusBar()->showMessage(
        QString("Сохранено как скан #%1 «%2»").arg(index).arg(name.trimmed()), 5000);
}

void MainWindow::onShowCloudClicked()
{
    if (!m_viewer) return;
    if (!m_meshViewerId.isEmpty()) {
        m_viewer->removePolygonMesh(m_meshViewerId.toStdString());
        m_meshViewerId.clear();
    }
    QMutexLocker locker(&m_cloudMutex);
    if (m_accumulatedCloud && !m_accumulatedCloud->empty()) {
        bool updated = m_viewer->updatePointCloud(m_accumulatedCloud, "accumulated");
        if (!updated) {
            m_viewer->addPointCloud(m_accumulatedCloud, "accumulated");
            m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "accumulated");
        }
    }
    m_vtkWidget->renderWindow()->Render();
}

// ========== Настройки → Параметры… ==========
void MainWindow::onShowSettingsDialog()
{
    SettingsDialog dlg(this);
    dlg.exec();
    // SettingsManager::settingsChanged уже эмитится при каждой записи,
    // так что подписчики (LiveCloudWindow и т.п.) получат обновления
    // автоматически. Существующие UI-виджеты на вкладках читают QSettings
    // только при запуске — при следующем взаимодействии они подхватят
    // новые значения через соответствующие click-handler'ы.
}

// ========== Поворотный стол / авто-сохранение ==========
void MainWindow::onTurntableToggled(bool enabled)
{
    if (!m_turntableTimer) return;

    if (!enabled) {
        m_turntableTimer->stop();
        if (m_turntableStatusLabel) {
            m_turntableStatusLabel->setText(
                QString("Остановлено. Сохранено %1 сканов").arg(m_turntableCaptured));
        }
        return;
    }

    // Предварительные условия: открытый проект обязателен, сканирование
    // опционально (если пользователь хочет просто snapshot-нуть текущее
    // накопленное облако несколько раз, это тоже валидно).
    if (!m_project || !m_project->isOpen()) {
        QMessageBox::warning(this, "Поворотный стол",
            "Сначала создайте или откройте проект (меню «Файл»).");
        m_turntableEnableChk->setChecked(false);
        return;
    }

    m_turntableCaptured = 0;
    const int intervalMs = m_turntableIntervalSpin->value() * 1000;
    m_turntableTimer->start(intervalMs);
    if (m_turntableStatusLabel) {
        m_turntableStatusLabel->setText(
            QString("Идёт: 0 / %1, интервал %2 с")
                .arg(m_turntableCountSpin->value())
                .arg(m_turntableIntervalSpin->value()));
    }
    qInfo() << "[Turntable] started: interval =" << intervalMs / 1000
            << "s, count =" << m_turntableCountSpin->value();
}

void MainWindow::onTurntableTick()
{
    if (!m_project || !m_project->isOpen()) {
        qWarning() << "[Turntable] project closed mid-run — stopping";
        m_turntableTimer->stop();
        if (m_turntableEnableChk) m_turntableEnableChk->setChecked(false);
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr snapshot;
    {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) {
            qWarning() << "[Turntable] tick: accumulated cloud empty, skipping";
            if (m_turntableStatusLabel) {
                m_turntableStatusLabel->setText(
                    QString("Пропуск тика (пустое облако), сохранено %1")
                        .arg(m_turntableCaptured));
            }
            return;
        }
        snapshot = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*m_accumulatedCloud);
        m_accumulatedCloud->clear();
    }

    const QString name = QString("turntable_%1_%2")
        .arg(m_turntableCaptured + 1, 3, 10, QChar('0'))
        .arg(QDateTime::currentDateTime().toString("HHmmss"));
    const int idx = m_project->addScan(snapshot, name);
    if (idx < 0) {
        qCritical() << "[Turntable] addScan failed:" << m_project->lastError();
        m_turntableTimer->stop();
        if (m_turntableEnableChk) m_turntableEnableChk->setChecked(false);
        QMessageBox::critical(this, "Поворотный стол",
            QString("Не удалось сохранить скан:\n%1").arg(m_project->lastError()));
        return;
    }
    ++m_turntableCaptured;

    // Обновляем GUI: счётчики облака, вьюер.
    emit cloudSizeChanged(0);
    if (m_viewer) m_viewer->removeAllPointClouds();

    const int target = m_turntableCountSpin->value();
    if (m_turntableStatusLabel) {
        m_turntableStatusLabel->setText(
            QString("Сохранено %1 / %2 (%3 точек в последнем скане)")
                .arg(m_turntableCaptured).arg(target).arg(snapshot->size()));
    }
    qInfo() << "[Turntable] saved scan" << m_turntableCaptured << "/" << target
            << "(" << snapshot->size() << "points) as index" << idx;

    if (m_turntableCaptured >= target) {
        m_turntableTimer->stop();
        if (m_turntableEnableChk) m_turntableEnableChk->setChecked(false);
        statusBar()->showMessage(
            QString("Поворотный стол: готово, сохранено %1 сканов").arg(m_turntableCaptured),
            5000);
        QMessageBox::information(this, "Поворотный стол",
            QString("Готово. Сохранено %1 сканов в проекте.\n"
                    "Перейдите на вкладку «Обработка» → «Объединить все сканы проекта», "
                    "чтобы склеить их через ICP.").arg(m_turntableCaptured));
    }
}
// ========== Ручное редактирование облака (лассо) ==========
namespace {

// Лимит снимков в undo-стеке. На больших облаках (несколько М точек)
// каждый снимок весит десятки МБ, 10 штук = верхний край «допустимого».
constexpr std::size_t kMaxEditUndo = 10;

} // namespace

void MainWindow::onLassoDeleteClicked()
{
    if (!m_lasso) return;
    if (m_lasso->isActive()) {
        // Повторный клик — воспринимаем как отмену.
        m_lasso->stop();
        m_lassoOp = LassoOp::None;
        m_lassoStatusLabel->setText("Лассо отменено");
        return;
    }

    QMutexLocker locker(&m_cloudMutex);
    if (!m_accumulatedCloud || m_accumulatedCloud->empty()) {
        locker.unlock();
        QMessageBox::information(this, "Лассо",
            "Текущее облако пустое — нечего редактировать.");
        return;
    }
    locker.unlock();

    m_lassoOp = LassoOp::Delete;
    m_lassoStatusLabel->setText(
        "Режим: удалить выделенное. Обведите область во вьюере, Esc — отмена.");
    m_lasso->start();
}

void MainWindow::onLassoKeepClicked()
{
    if (!m_lasso) return;
    if (m_lasso->isActive()) {
        m_lasso->stop();
        m_lassoOp = LassoOp::None;
        m_lassoStatusLabel->setText("Лассо отменено");
        return;
    }

    QMutexLocker locker(&m_cloudMutex);
    if (!m_accumulatedCloud || m_accumulatedCloud->empty()) {
        locker.unlock();
        QMessageBox::information(this, "Лассо",
            "Текущее облако пустое — нечего редактировать.");
        return;
    }
    locker.unlock();

    m_lassoOp = LassoOp::Keep;
    m_lassoStatusLabel->setText(
        "Режим: оставить выделенное. Обведите область во вьюере, Esc — отмена.");
    m_lasso->start();
}

void MainWindow::onLassoCanceled()
{
    m_lassoOp = LassoOp::None;
    if (m_lassoStatusLabel) m_lassoStatusLabel->setText("Лассо отменено");
}

void MainWindow::onLassoCompleted(const QPolygonF &polygonWidget)
{
    if (m_lassoOp == LassoOp::None || !m_vtkRenderer || !m_vtkWidget) {
        m_lassoOp = LassoOp::None;
        return;
    }
    if (polygonWidget.size() < 3) {
        m_lassoOp = LassoOp::None;
        if (m_lassoStatusLabel) m_lassoStatusLabel->setText(
            "Слишком маленький контур, операция отменена");
        return;
    }

    // Переводим полигон в display-систему VTK (физические пиксели,
    // origin — низ-лево), чтобы он совпал с координатной системой, в
    // которой мы будем проецировать точки облака.
    const double dpr = m_vtkWidget->devicePixelRatioF();
    const double wLogicalH = m_vtkWidget->height();
    QPolygonF polyDisplay;
    polyDisplay.reserve(polygonWidget.size());
    for (const QPointF &p : polygonWidget) {
        polyDisplay << QPointF(p.x() * dpr, (wLogicalH - p.y()) * dpr);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr snapshot;
    {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud || m_accumulatedCloud->empty()) {
            m_lassoOp = LassoOp::None;
            return;
        }
        // Снимок до правки — для undo.
        snapshot = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*m_accumulatedCloud);
    }

    // Проецируем точки по текущей камере/viewport. WorldToDisplay не
    // потокобезопасен на общем renderer, так что работаем в GUI-потоке
    // (этот слот и так сюда приходит).
    const std::size_t N = snapshot->size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    result->reserve(N);

    const bool keep = (m_lassoOp == LassoOp::Keep);
    std::size_t removed = 0;

    for (std::size_t i = 0; i < N; ++i) {
        const auto &pt = snapshot->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            // Невалидные точки в любом случае отбрасываем — незачем их
            // тащить через правку.
            ++removed;
            continue;
        }
        double world[4] = {pt.x, pt.y, pt.z, 1.0};
        m_vtkRenderer->SetWorldPoint(world);
        m_vtkRenderer->WorldToDisplay();
        double disp[3];
        m_vtkRenderer->GetDisplayPoint(disp);

        const QPointF d(disp[0], disp[1]);
        const bool inside = polyDisplay.containsPoint(d, Qt::OddEvenFill);
        const bool accept = keep ? inside : !inside;
        if (accept) {
            result->push_back(pt);
        } else {
            ++removed;
        }
    }

    if (result->size() == N) {
        m_lassoOp = LassoOp::None;
        if (m_lassoStatusLabel) m_lassoStatusLabel->setText(
            "В полигон не попала ни одна точка — облако не изменилось");
        return;
    }

    result->width = result->size();
    result->height = 1;
    result->is_dense = true;

    // Push snapshot → undo stack (ограниченный).
    m_editUndo.push_back(snapshot);
    while (m_editUndo.size() > kMaxEditUndo) {
        m_editUndo.pop_front();
    }
    if (m_undoEditBtn) m_undoEditBtn->setEnabled(true);

    {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud) {
            m_accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        }
        *m_accumulatedCloud = *result;
    }
    emit cloudSizeChanged(static_cast<int>(result->size()));

    // Если в вьюере висит меш — убираем, чтобы результат правки было видно.
    if (m_viewer && !m_meshViewerId.isEmpty()) {
        m_viewer->removePolygonMesh(m_meshViewerId.toStdString());
        m_meshViewerId.clear();
    }
    updateViewer();

    if (m_lassoStatusLabel) {
        if (keep) {
            m_lassoStatusLabel->setText(
                QString("Оставлено %1 точек, удалено %2")
                    .arg(result->size())
                    .arg(removed));
        } else {
            m_lassoStatusLabel->setText(
                QString("Удалено %1 точек, осталось %2")
                    .arg(removed)
                    .arg(result->size()));
        }
    }
    qInfo() << "[Lasso]" << (keep ? "keep" : "delete")
            << "removed" << removed << "points; result size =" << result->size();
    m_lassoOp = LassoOp::None;
}

void MainWindow::onUndoEditClicked()
{
    if (m_editUndo.empty()) {
        if (m_lassoStatusLabel) m_lassoStatusLabel->setText("Нечего отменять");
        if (m_undoEditBtn) m_undoEditBtn->setEnabled(false);
        return;
    }
    auto snap = m_editUndo.back();
    m_editUndo.pop_back();

    {
        QMutexLocker locker(&m_cloudMutex);
        if (!m_accumulatedCloud) {
            m_accumulatedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        }
        *m_accumulatedCloud = *snap;
    }
    emit cloudSizeChanged(static_cast<int>(snap->size()));
    updateViewer();

    if (m_undoEditBtn) m_undoEditBtn->setEnabled(!m_editUndo.empty());
    if (m_lassoStatusLabel) m_lassoStatusLabel->setText(
        QString("Правка отменена, восстановлено %1 точек").arg(snap->size()));
    qInfo() << "[Lasso] undo: restored" << snap->size() << "points, undo stack ="
            << m_editUndo.size();
}

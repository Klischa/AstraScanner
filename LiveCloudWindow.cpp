#include "LiveCloudWindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QStatusBar>
#include <QAction>
#include <QMenuBar>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <QDebug>

LiveCloudWindow::LiveCloudWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUI();
    setupVisualizer();

    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &LiveCloudWindow::onTimerTimeout);

    setWindowTitle("Live Cloud Viewer - AstraScanner");
    resize(800, 600);
}

LiveCloudWindow::~LiveCloudWindow()
{
    stopUpdating();
}

void LiveCloudWindow::setupUI()
{
    // Создаем центральный виджет
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // Панель управления
    QHBoxLayout *controlLayout = new QHBoxLayout();

    QPushButton *startBtn = new QPushButton("Start Live View", this);
    QPushButton *stopBtn = new QPushButton("Stop", this);

    QLabel *intervalLabel = new QLabel("Update interval (ms):", this);
    QSpinBox *intervalSpin = new QSpinBox(this);
    intervalSpin->setRange(100, 2000);
    intervalSpin->setValue(500);
    intervalSpin->setSingleStep(100);

    controlLayout->addWidget(startBtn);
    controlLayout->addWidget(stopBtn);
    controlLayout->addWidget(intervalLabel);
    controlLayout->addWidget(intervalSpin);
    controlLayout->addStretch();

    mainLayout->addLayout(controlLayout);

    // VTK виджет
    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    mainLayout->addWidget(m_vtkWidget);

    // Статус бар
    QStatusBar *statusBar = new QStatusBar(this);
    setStatusBar(statusBar);

    QLabel *statusLabel = new QLabel("Ready", this);
    statusBar->addWidget(statusLabel);

    // Меню
    QMenuBar *menuBar = new QMenuBar(this);
    setMenuBar(menuBar);

    QMenu *viewMenu = menuBar->addMenu("View");
    QAction *resetViewAction = viewMenu->addAction("Reset View");
    QAction *fitToScreenAction = viewMenu->addAction("Fit to Screen");

    // Подключения
    connect(startBtn, &QPushButton::clicked, this, &LiveCloudWindow::startUpdating);
    connect(stopBtn, &QPushButton::clicked, this, &LiveCloudWindow::stopUpdating);
    connect(intervalSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        setUpdateInterval(value);
    });

    connect(resetViewAction, &QAction::triggered, this, [this]() {
        m_renderer->ResetCamera();
        m_renderWindow->Render();
    });

    connect(fitToScreenAction, &QAction::triggered, this, [this]() {
        m_renderer->ResetCamera();
        m_renderer->GetActiveCamera()->Zoom(1.0);
        m_renderWindow->Render();
    });
}

void LiveCloudWindow::setupVisualizer()
{
    // Создаём правильный тип render window
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_vtkWidget->setRenderWindow(renderWindow);

    // Создаем renderer
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(m_renderer);
    m_renderer->SetBackground(0.1, 0.1, 0.1);

    // Настраиваем камеру
    m_renderer->GetActiveCamera()->SetPosition(0, 0, 2);
    m_renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
    m_renderer->GetActiveCamera()->SetViewUp(0, 1, 0);

    // Создаем точки и цвета
    m_points = vtkSmartPointer<vtkPoints>::New();
    m_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    m_colors->SetNumberOfComponents(3);
    m_colors->SetName("Colors");

    // Создаем polydata
    m_polyData = vtkSmartPointer<vtkPolyData>::New();
    m_polyData->SetPoints(m_points);
    m_polyData->GetPointData()->SetScalars(m_colors);

    // Создаем glyph filter
    m_glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    m_glyphFilter->SetInputData(m_polyData);

    // Создаем mapper
    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputConnection(m_glyphFilter->GetOutputPort());

    // Создаем actor
    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    m_actor->GetProperty()->SetPointSize(2);

    // Добавляем actor в renderer
    m_renderer->AddActor(m_actor);

    // Настраиваем interactor
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = renderWindow->GetInteractor();
    if (interactor) {
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        interactor->SetInteractorStyle(style);
    }

    renderWindow->Render();
}

void LiveCloudWindow::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    QMutexLocker locker(&m_cloudMutex);
    m_currentCloud = cloud;
}

void LiveCloudWindow::startUpdating()
{
    if (!m_updateTimer->isActive()) {
        m_updateTimer->start();
        statusBar()->showMessage("Live view started", 2000);
    }
}

void LiveCloudWindow::stopUpdating()
{
    if (m_updateTimer->isActive()) {
        m_updateTimer->stop();
        statusBar()->showMessage("Live view stopped", 2000);
    }
}

void LiveCloudWindow::onTimerTimeout()
{
    emit cloudRequested();

    // Обновляем визуализацию если есть новое облако
    QMutexLocker locker(&m_cloudMutex);
    if (m_currentCloud && !m_currentCloud->empty()) {
        // Очищаем предыдущие данные
        m_points->Reset();
        m_colors->Reset();

        // Добавляем точки
        for (const auto& point : m_currentCloud->points) {
            m_points->InsertNextPoint(point.x, point.y, point.z);
            m_colors->InsertNextTuple3(point.r, point.g, point.b);
        }

        m_polyData->Modified();
        m_renderWindow->Render();

        // Обновляем статус
        statusBar()->showMessage(QString("Points: %1").arg(m_currentCloud->size()), 1000);
    }
}
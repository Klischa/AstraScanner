#ifndef LIVE_CLOUD_WINDOW_H
#define LIVE_CLOUD_WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QMutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtk-9.3/QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkLookupTable.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkPointData.h>

class LiveCloudWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit LiveCloudWindow(QWidget *parent = nullptr);
    ~LiveCloudWindow();

    void updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    void setUpdateInterval(int ms) { m_updateTimer->setInterval(ms); }

public slots:
    void startUpdating();
    void stopUpdating();

signals:
    void cloudRequested();

private slots:
    void onTimerTimeout();

private:
    void setupUI();
    void setupVisualizer();

    QVTKOpenGLNativeWidget *m_vtkWidget = nullptr;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkPoints> m_points;
    vtkSmartPointer<vtkUnsignedCharArray> m_colors;
    vtkSmartPointer<vtkPolyData> m_polyData;
    vtkSmartPointer<vtkVertexGlyphFilter> m_glyphFilter;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkLookupTable> m_lut;

    QTimer *m_updateTimer = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_currentCloud;
    QMutex m_cloudMutex;
};

#endif // LIVE_CLOUD_WINDOW_H
#ifndef POINTCLOUDEDITOR_H
#define POINTCLOUDEDITOR_H

#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <stack>
#include <vector>
#include <array>
#include <functional>

class QVTKOpenGLNativeWidget;

// ---------------------------------------------------------------------------
// LassoInteractorStyle — VTK interactor style: при зажатой ЛКМ рисует
// произвольный замкнутый контур (лассо) поверх 3D-вьюпорта. При отпускании
// кнопки вызывает callback с полигоном в экранных координатах.
// ---------------------------------------------------------------------------
class LassoInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static LassoInteractorStyle *New();
    vtkTypeMacro(LassoInteractorStyle, vtkInteractorStyleTrackballCamera);

    void OnLeftButtonDown() override;
    void OnMouseMove() override;
    void OnLeftButtonUp() override;

    // Callback, вызываемый при завершении лассо. Аргумент — полигон в
    // экранных координатах (x, y).
    using LassoDoneCallback = std::function<void(const std::vector<std::pair<int,int>> &)>;
    void setLassoDoneCallback(LassoDoneCallback cb) { m_callback = std::move(cb); }

    bool isDrawing() const { return m_drawing; }

private:
    LassoInteractorStyle() = default;
    ~LassoInteractorStyle() override = default;

    bool m_drawing = false;
    std::vector<std::pair<int,int>> m_polyline;  // экранные координаты
    LassoDoneCallback m_callback;
};

// ---------------------------------------------------------------------------
// PointCloudEditor — ручное редактирование облака точек через лассо.
//
// Режим работы:
//   1. setEditMode(true) — interactor style подменяется на LassoInteractorStyle.
//      Пользователь рисует замкнутый контур мышью.
//   2. Все точки, чья проекция на экран попадает внутрь контура, считаются
//      выделенными и подсвечиваются красным.
//   3. Пользователь выполняет операцию: удалить / оставить только / инвертировать.
//   4. Каждая мутация сохраняет предыдущее состояние в undo-стеке (до 10).
//   5. setEditMode(false) — возвращает trackball-камеру, снимает выделение.
// ---------------------------------------------------------------------------
class PointCloudEditor : public QObject
{
    Q_OBJECT
public:
    explicit PointCloudEditor(QObject *parent = nullptr);

    // Привязка к визуализатору. Вызывать один раз после setupVisualizer.
    void attach(pcl::visualization::PCLVisualizer::Ptr viewer,
                QVTKOpenGLNativeWidget *vtkWidget);

    // Установить облако для редактирования (не владеет, работает по указателю).
    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    // Режим редактирования.
    void setEditMode(bool enabled);
    bool isEditMode() const { return m_editMode; }

    // --- Операции над выделением ---
    void deleteSelected();       // удалить выделенные точки
    void cropToSelected();       // оставить только выделенные
    void invertSelection();
    void selectAll();
    void clearSelection();

    void undo();
    bool canUndo() const;

    int selectedCount() const { return static_cast<int>(m_selectedIndices.size()); }

    static constexpr int kMaxUndoLevels = 10;

signals:
    void cloudModified(int newSize);
    void selectionChanged(int selectedCount);
    void editModeChanged(bool enabled);

private:
    // Обработчик завершения лассо.
    void onLassoDone(const std::vector<std::pair<int,int>> &polygon);
    // Проецирует 3D-точку в экранные координаты.
    bool projectToScreen(float x, float y, float z, int &sx, int &sy);
    // Point-in-polygon (ray casting).
    static bool pointInPolygon(int px, int py,
                               const std::vector<std::pair<int,int>> &poly);

    void refreshViewer();
    void pushUndo();

    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    QVTKOpenGLNativeWidget *m_vtkWidget = nullptr;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
    // Оригинальные цвета (до highlight). Размер = cloud->size().
    std::vector<std::array<uint8_t, 3>> m_originalColors;

    std::vector<int> m_selectedIndices;

    std::stack<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_undoStack;

    bool m_editMode = false;

    vtkSmartPointer<LassoInteractorStyle> m_lassoStyle;
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> m_defaultStyle;

    static constexpr uint8_t kHighlightR = 255;
    static constexpr uint8_t kHighlightG = 50;
    static constexpr uint8_t kHighlightB = 50;
};

#endif // POINTCLOUDEDITOR_H

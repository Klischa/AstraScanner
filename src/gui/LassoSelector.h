#ifndef LASSOSELECTOR_H
#define LASSOSELECTOR_H

#include <QObject>
#include <QPolygonF>
#include <vtkSmartPointer.h>

class QVTKOpenGLNativeWidget;
class vtkRenderer;
class vtkActor2D;
class vtkPoints;
class vtkCellArray;
class vtkPolyData;

// Свободное (лассо) выделение области во вьюере PCL/VTK для ручного
// редактирования облака точек.
//
// LassoSelector вешает event filter на QVTKOpenGLNativeWidget и рисует
// живой контур прямо в VTK-рендерере через vtkActor2D в display-
// координатах. На release кнопки мыши класс эмитит lassoCompleted с
// полигоном в координатах виджета (лог. пиксели, origin — левый-верх),
// который потребитель (MainWindow) потом сам тестирует против проекции
// каждой точки облака.
//
// Использование:
//    m_lasso = new LassoSelector(m_vtkWidget, m_viewer->getRendererCollection()
//                                ->GetFirstRenderer(), this);
//    connect(m_lasso, &LassoSelector::lassoCompleted, this, &MainWindow::onLasso);
//    m_lasso->start();  // и затем стандартный Qt event loop
class LassoSelector : public QObject
{
    Q_OBJECT
public:
    explicit LassoSelector(QVTKOpenGLNativeWidget *widget,
                           vtkRenderer *renderer,
                           QObject *parent = nullptr);
    ~LassoSelector() override;

    void start();
    void stop();
    bool isActive() const { return m_active; }

signals:
    // Полигон в координатах виджета (логические пиксели, origin — левый-
    // верхний угол виджета). Размер всегда ≥ 3.
    void lassoCompleted(const QPolygonF &polygonWidget);
    void lassoCanceled();

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    void addPoint(const QPointF &pt);
    void rebuildOverlay();
    void hideOverlay();
    void requestRender();

    QVTKOpenGLNativeWidget *m_widget = nullptr;
    vtkSmartPointer<vtkRenderer> m_renderer;

    // Оверлей — vtkActor2D с vtkPolyData (точки в display-системе). Не
    // пересоздаём, только меняем содержимое, чтобы при старте следующего
    // лассо не было мигания.
    vtkSmartPointer<vtkActor2D>   m_overlayActor;
    vtkSmartPointer<vtkPoints>    m_overlayPoints;
    vtkSmartPointer<vtkCellArray> m_overlayLines;
    vtkSmartPointer<vtkPolyData>  m_overlayData;

    QPolygonF m_poly;
    bool m_active = false;
    bool m_dragging = false;
};

#endif // LASSOSELECTOR_H

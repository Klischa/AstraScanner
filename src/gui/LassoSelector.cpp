#include "LassoSelector.h"

#include <QVTKOpenGLNativeWidget.h>

#include <QApplication>
#include <QDebug>
#include <QKeyEvent>
#include <QMouseEvent>

#include <vtkActor2D.h>
#include <vtkCellArray.h>
#include <vtkCoordinate.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkPoints.h>
#include <vtkProperty2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

namespace {

// Минимальное расстояние между последовательными точками полигона,
// чтобы не забить список десятками тысяч почти-дублирующихся узлов при
// медленном движении мыши. 2 px — видно на глаз, но не мусор.
constexpr double kMinSegmentPx = 2.0;

// Минимум вершин, который считаем «валидным» лассо.
constexpr int kMinPolygonVertices = 3;

// Конвертация точки в координатах Qt-виджета (логические пиксели,
// origin — верх-лево) в display-координаты VTK (физические пиксели,
// origin — низ-лево).
QPointF widgetToVtkDisplay(const QPointF &p, QWidget *w)
{
    const double dpr = w ? w->devicePixelRatioF() : 1.0;
    const double hLogical = w ? w->height() : 0.0;
    return QPointF(p.x() * dpr, (hLogical - p.y()) * dpr);
}

} // namespace

LassoSelector::LassoSelector(QVTKOpenGLNativeWidget *widget,
                             vtkRenderer *renderer,
                             QObject *parent)
    : QObject(parent), m_widget(widget), m_renderer(renderer)
{
    m_overlayPoints = vtkSmartPointer<vtkPoints>::New();
    m_overlayLines  = vtkSmartPointer<vtkCellArray>::New();
    m_overlayData   = vtkSmartPointer<vtkPolyData>::New();
    m_overlayData->SetPoints(m_overlayPoints);
    m_overlayData->SetLines(m_overlayLines);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    mapper->SetInputData(m_overlayData);
    // Координаты точек полигона — в display-системе (физ. пиксели,
    // origin низ-лево). vtkActor2D без явного vtkCoordinate по умолчанию
    // интерпретирует вход как display coords, так что SetTransformCoordinate
    // не обязателен, но укажу явно ради читаемости.
    auto coord = vtkSmartPointer<vtkCoordinate>::New();
    coord->SetCoordinateSystemToDisplay();
    mapper->SetTransformCoordinate(coord);

    m_overlayActor = vtkSmartPointer<vtkActor2D>::New();
    m_overlayActor->SetMapper(mapper);
    m_overlayActor->GetProperty()->SetColor(1.0, 0.2, 0.0); // ярко-красный
    m_overlayActor->GetProperty()->SetLineWidth(2.0);
    m_overlayActor->SetVisibility(0);

    if (m_renderer) {
        m_renderer->AddActor2D(m_overlayActor);
    }
}

LassoSelector::~LassoSelector()
{
    stop();
    if (m_renderer && m_overlayActor) {
        m_renderer->RemoveActor2D(m_overlayActor);
    }
}

void LassoSelector::start()
{
    if (m_active || !m_widget) return;
    m_active = true;
    m_dragging = false;
    m_poly.clear();
    hideOverlay();
    m_widget->installEventFilter(this);
    m_widget->setCursor(Qt::CrossCursor);
    m_widget->setFocus(Qt::OtherFocusReason);
}

void LassoSelector::stop()
{
    if (!m_active) return;
    m_active = false;
    m_dragging = false;
    m_poly.clear();
    hideOverlay();
    if (m_widget) {
        m_widget->removeEventFilter(this);
        m_widget->unsetCursor();
    }
    requestRender();
}

bool LassoSelector::eventFilter(QObject *obj, QEvent *event)
{
    if (!m_active || obj != m_widget) {
        return QObject::eventFilter(obj, event);
    }

    switch (event->type()) {
    case QEvent::MouseButtonPress: {
        auto *me = static_cast<QMouseEvent *>(event);
        if (me->button() != Qt::LeftButton) break;
        m_poly.clear();
        m_dragging = true;
        addPoint(me->position());
        rebuildOverlay();
        requestRender();
        return true; // съедаем — VTK камера не должна крутиться под лассо
    }
    case QEvent::MouseMove: {
        if (!m_dragging) break;
        auto *me = static_cast<QMouseEvent *>(event);
        addPoint(me->position());
        rebuildOverlay();
        requestRender();
        return true;
    }
    case QEvent::MouseButtonRelease: {
        auto *me = static_cast<QMouseEvent *>(event);
        if (me->button() != Qt::LeftButton) break;
        if (!m_dragging) break;
        m_dragging = false;

        if (m_poly.size() < kMinPolygonVertices) {
            emit lassoCanceled();
            stop();
            return true;
        }

        QPolygonF finished = m_poly;
        // Закрываем полигон, если пользователь не замкнул сам.
        if (finished.first() != finished.last()) {
            finished << finished.first();
        }
        stop();
        emit lassoCompleted(finished);
        return true;
    }
    case QEvent::KeyPress: {
        auto *ke = static_cast<QKeyEvent *>(event);
        if (ke->key() == Qt::Key_Escape) {
            emit lassoCanceled();
            stop();
            return true;
        }
        break;
    }
    default:
        break;
    }
    return QObject::eventFilter(obj, event);
}

void LassoSelector::addPoint(const QPointF &pt)
{
    if (!m_poly.isEmpty()) {
        const QPointF d = pt - m_poly.last();
        if (std::hypot(d.x(), d.y()) < kMinSegmentPx) return;
    }
    m_poly << pt;
}

void LassoSelector::rebuildOverlay()
{
    if (!m_renderer || !m_widget) return;

    m_overlayPoints->Reset();
    m_overlayLines->Reset();

    const vtkIdType n = static_cast<vtkIdType>(m_poly.size());
    if (n < 2) {
        m_overlayActor->SetVisibility(0);
        return;
    }

    m_overlayPoints->SetNumberOfPoints(n);
    for (vtkIdType i = 0; i < n; ++i) {
        const QPointF d = widgetToVtkDisplay(m_poly.at(i), m_widget);
        m_overlayPoints->SetPoint(i, d.x(), d.y(), 0.0);
    }

    // Один polyline через все точки. Контур не замыкаем во время drag,
    // чтобы пользователю было видно, где сейчас «курсор».
    m_overlayLines->InsertNextCell(n);
    for (vtkIdType i = 0; i < n; ++i) {
        m_overlayLines->InsertCellPoint(i);
    }

    m_overlayData->Modified();
    m_overlayActor->SetVisibility(1);
}

void LassoSelector::hideOverlay()
{
    if (!m_overlayActor) return;
    m_overlayActor->SetVisibility(0);
    if (m_overlayPoints) m_overlayPoints->Reset();
    if (m_overlayLines)  m_overlayLines->Reset();
    if (m_overlayData)   m_overlayData->Modified();
}

void LassoSelector::requestRender()
{
    if (!m_widget) return;
    if (auto *win = m_widget->renderWindow()) win->Render();
}

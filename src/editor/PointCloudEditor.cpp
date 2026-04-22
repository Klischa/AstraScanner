#include "PointCloudEditor.h"

#include <QDebug>
#include <algorithm>
#include <unordered_set>
#include <cmath>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkCoordinate.h>
#include <vtkObjectFactory.h>

// ===================== LassoInteractorStyle =====================

vtkStandardNewMacro(LassoInteractorStyle);

void LassoInteractorStyle::OnLeftButtonDown()
{
    m_drawing = true;
    m_polyline.clear();
    int *pos = this->GetInteractor()->GetEventPosition();
    m_polyline.push_back({pos[0], pos[1]});
}

void LassoInteractorStyle::OnMouseMove()
{
    if (m_drawing) {
        int *pos = this->GetInteractor()->GetEventPosition();
        m_polyline.push_back({pos[0], pos[1]});
    } else {
        vtkInteractorStyleTrackballCamera::OnMouseMove();
    }
}

void LassoInteractorStyle::OnLeftButtonUp()
{
    if (!m_drawing) return;
    m_drawing = false;
    if (m_polyline.size() >= 3 && m_callback) {
        m_callback(m_polyline);
    }
    m_polyline.clear();
}

// ===================== PointCloudEditor =====================

PointCloudEditor::PointCloudEditor(QObject *parent) : QObject(parent) {}

void PointCloudEditor::attach(pcl::visualization::PCLVisualizer::Ptr viewer,
                              QVTKOpenGLNativeWidget *vtkWidget)
{
    m_viewer = viewer;
    m_vtkWidget = vtkWidget;
    m_defaultStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    m_lassoStyle = vtkSmartPointer<LassoInteractorStyle>::New();
    m_lassoStyle->setLassoDoneCallback(
        [this](const std::vector<std::pair<int,int>> &poly) { onLassoDone(poly); });
}

void PointCloudEditor::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    m_cloud = cloud;
    m_selectedIndices.clear();
    while (!m_undoStack.empty()) m_undoStack.pop();
    m_originalColors.clear();
    if (m_cloud) {
        m_originalColors.reserve(m_cloud->size());
        for (const auto &pt : m_cloud->points)
            m_originalColors.push_back({pt.r, pt.g, pt.b});
    }
    emit selectionChanged(0);
}

void PointCloudEditor::setEditMode(bool enabled)
{
    if (m_editMode == enabled) return;
    m_editMode = enabled;
    if (!m_vtkWidget) { qWarning() << "[Editor] vtkWidget not attached"; return; }
    auto interactor = m_vtkWidget->interactor();
    if (!interactor) { qWarning() << "[Editor] no interactor"; return; }

    if (m_editMode) {
        interactor->SetInteractorStyle(m_lassoStyle);
        qInfo() << "[Editor] Edit mode ON";
    } else {
        interactor->SetInteractorStyle(m_defaultStyle);
        clearSelection();
        qInfo() << "[Editor] Edit mode OFF";
    }
    emit editModeChanged(m_editMode);
}

// ==================== Operations ====================

void PointCloudEditor::deleteSelected()
{
    if (!m_cloud || m_selectedIndices.empty()) return;
    pushUndo();
    std::unordered_set<int> rm(m_selectedIndices.begin(), m_selectedIndices.end());
    auto nc = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    nc->reserve(m_cloud->size() - rm.size());
    std::vector<std::array<uint8_t,3>> nc_colors;
    nc_colors.reserve(nc->capacity());
    for (int i = 0; i < static_cast<int>(m_cloud->size()); ++i) {
        if (rm.count(i)) continue;
        auto pt = m_cloud->points[i];
        if (i < static_cast<int>(m_originalColors.size())) {
            pt.r = m_originalColors[i][0]; pt.g = m_originalColors[i][1]; pt.b = m_originalColors[i][2];
        }
        nc->push_back(pt);
        nc_colors.push_back(i < static_cast<int>(m_originalColors.size())
            ? m_originalColors[i] : std::array<uint8_t,3>{pt.r, pt.g, pt.b});
    }
    nc->width = nc->size(); nc->height = 1; nc->is_dense = m_cloud->is_dense;
    const int removed = static_cast<int>(rm.size());
    *m_cloud = *nc; m_originalColors = std::move(nc_colors); m_selectedIndices.clear();
    qInfo() << "[Editor] Deleted" << removed << "pts, remaining:" << m_cloud->size();
    emit selectionChanged(0);
    emit cloudModified(static_cast<int>(m_cloud->size()));
    refreshViewer();
}

void PointCloudEditor::cropToSelected()
{
    if (!m_cloud || m_selectedIndices.empty()) return;
    pushUndo();
    std::vector<int> sorted = m_selectedIndices;
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    auto nc = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    nc->reserve(sorted.size());
    std::vector<std::array<uint8_t,3>> nc_colors;
    nc_colors.reserve(sorted.size());
    for (int idx : sorted) {
        if (idx < 0 || idx >= static_cast<int>(m_cloud->size())) continue;
        auto pt = m_cloud->points[idx];
        if (idx < static_cast<int>(m_originalColors.size())) {
            pt.r = m_originalColors[idx][0]; pt.g = m_originalColors[idx][1]; pt.b = m_originalColors[idx][2];
        }
        nc->push_back(pt);
        nc_colors.push_back(idx < static_cast<int>(m_originalColors.size())
            ? m_originalColors[idx] : std::array<uint8_t,3>{pt.r, pt.g, pt.b});
    }
    nc->width = nc->size(); nc->height = 1; nc->is_dense = m_cloud->is_dense;
    *m_cloud = *nc; m_originalColors = std::move(nc_colors); m_selectedIndices.clear();
    qInfo() << "[Editor] Cropped to" << m_cloud->size() << "pts";
    emit selectionChanged(0);
    emit cloudModified(static_cast<int>(m_cloud->size()));
    refreshViewer();
}

void PointCloudEditor::invertSelection()
{
    if (!m_cloud) return;
    std::unordered_set<int> cur(m_selectedIndices.begin(), m_selectedIndices.end());
    m_selectedIndices.clear();
    for (int i = 0; i < static_cast<int>(m_cloud->size()); ++i)
        if (cur.count(i) == 0) m_selectedIndices.push_back(i);
    emit selectionChanged(static_cast<int>(m_selectedIndices.size()));
    refreshViewer();
}

void PointCloudEditor::selectAll()
{
    if (!m_cloud) return;
    m_selectedIndices.resize(m_cloud->size());
    for (int i = 0; i < static_cast<int>(m_cloud->size()); ++i) m_selectedIndices[i] = i;
    emit selectionChanged(static_cast<int>(m_selectedIndices.size()));
    refreshViewer();
}

void PointCloudEditor::clearSelection()
{
    if (m_selectedIndices.empty()) return;
    if (m_cloud) {
        for (int idx : m_selectedIndices) {
            if (idx >= 0 && idx < static_cast<int>(m_cloud->size()) &&
                idx < static_cast<int>(m_originalColors.size())) {
                m_cloud->points[idx].r = m_originalColors[idx][0];
                m_cloud->points[idx].g = m_originalColors[idx][1];
                m_cloud->points[idx].b = m_originalColors[idx][2];
            }
        }
    }
    m_selectedIndices.clear();
    emit selectionChanged(0);
    refreshViewer();
}

void PointCloudEditor::undo()
{
    if (m_undoStack.empty() || !m_cloud) return;
    *m_cloud = *m_undoStack.top(); m_undoStack.pop();
    m_selectedIndices.clear();
    m_originalColors.clear();
    m_originalColors.reserve(m_cloud->size());
    for (const auto &pt : m_cloud->points) m_originalColors.push_back({pt.r, pt.g, pt.b});
    const int sz = static_cast<int>(m_cloud->size());
    qInfo() << "[Editor] Undo:" << sz << "pts";
    emit selectionChanged(0); emit cloudModified(sz); refreshViewer();
}

bool PointCloudEditor::canUndo() const { return !m_undoStack.empty(); }

// ==================== Internal ====================

void PointCloudEditor::pushUndo()
{
    if (!m_cloud) return;
    auto snap = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*m_cloud);
    for (int i = 0; i < static_cast<int>(snap->size()) &&
         i < static_cast<int>(m_originalColors.size()); ++i) {
        snap->points[i].r = m_originalColors[i][0];
        snap->points[i].g = m_originalColors[i][1];
        snap->points[i].b = m_originalColors[i][2];
    }
    m_undoStack.push(snap);
    if (static_cast<int>(m_undoStack.size()) > kMaxUndoLevels) {
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> tmp;
        while (!m_undoStack.empty()) { tmp.push_back(m_undoStack.top()); m_undoStack.pop(); }
        for (int i = kMaxUndoLevels - 1; i >= 0; --i) m_undoStack.push(tmp[i]);
    }
}

void PointCloudEditor::refreshViewer()
{
    if (!m_viewer || !m_cloud) return;
    std::unordered_set<int> sel(m_selectedIndices.begin(), m_selectedIndices.end());
    for (int i = 0; i < static_cast<int>(m_cloud->size()); ++i) {
        if (sel.count(i)) {
            m_cloud->points[i].r = kHighlightR;
            m_cloud->points[i].g = kHighlightG;
            m_cloud->points[i].b = kHighlightB;
        } else if (i < static_cast<int>(m_originalColors.size())) {
            m_cloud->points[i].r = m_originalColors[i][0];
            m_cloud->points[i].g = m_originalColors[i][1];
            m_cloud->points[i].b = m_originalColors[i][2];
        }
    }
    bool ok = m_viewer->updatePointCloud(m_cloud, "accumulated");
    if (!ok) {
        m_viewer->addPointCloud(m_cloud, "accumulated");
        m_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "accumulated");
    }
    if (m_vtkWidget) m_vtkWidget->renderWindow()->Render();
}

// ==================== Lasso -> selection ====================

void PointCloudEditor::onLassoDone(const std::vector<std::pair<int,int>> &polygon)
{
    if (!m_cloud || m_cloud->empty() || polygon.size() < 3) return;
    m_selectedIndices.clear();
    for (int i = 0; i < static_cast<int>(m_cloud->size()); ++i) {
        const auto &pt = m_cloud->points[i];
        if (!std::isfinite(pt.x)) continue;
        int sx, sy;
        if (!projectToScreen(pt.x, pt.y, pt.z, sx, sy)) continue;
        if (pointInPolygon(sx, sy, polygon)) m_selectedIndices.push_back(i);
    }
    qInfo() << "[Editor] Lasso selected" << m_selectedIndices.size()
            << "of" << m_cloud->size() << "pts";
    emit selectionChanged(static_cast<int>(m_selectedIndices.size()));
    refreshViewer();
}

bool PointCloudEditor::projectToScreen(float x, float y, float z, int &sx, int &sy)
{
    if (!m_viewer) return false;
    auto renderer = m_viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer();
    if (!renderer) return false;
    auto coord = vtkSmartPointer<vtkCoordinate>::New();
    coord->SetCoordinateSystemToWorld();
    coord->SetValue(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
    int *scr = coord->GetComputedDisplayValue(renderer);
    sx = scr[0]; sy = scr[1];
    return true;
}

bool PointCloudEditor::pointInPolygon(int px, int py,
                                      const std::vector<std::pair<int,int>> &poly)
{
    bool inside = false;
    const int n = static_cast<int>(poly.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
        int xi = poly[i].first, yi = poly[i].second;
        int xj = poly[j].first, yj = poly[j].second;
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi + 1e-10) + xi))
            inside = !inside;
    }
    return inside;
}

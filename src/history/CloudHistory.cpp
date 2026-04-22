#include "CloudHistory.h"
#include <QDebug>

CloudHistory::CloudHistory(QObject *parent) : QObject(parent) {}

void CloudHistory::pushState(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             const QString &description)
{
    if (!cloud || cloud->empty()) return;

    Snapshot snap;
    snap.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*cloud);
    snap.description = description;
    m_undoStack.push_back(std::move(snap));

    // Новое действие сбрасывает redo-стек.
    m_redoStack.clear();

    // Ограничиваем глубину.
    while (static_cast<int>(m_undoStack.size()) > m_maxDepth) {
        m_undoStack.pop_front();
    }

    qDebug() << "[CloudHistory] push:" << description
             << "| undo depth:" << m_undoStack.size();
    emit historyChanged();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudHistory::undo(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &currentCloud)
{
    if (m_undoStack.empty()) return nullptr;

    // Сохраняем текущее состояние в redo.
    if (currentCloud && !currentCloud->empty()) {
        Snapshot redoSnap;
        redoSnap.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*currentCloud);
        redoSnap.description = m_undoStack.back().description;
        m_redoStack.push_back(std::move(redoSnap));
    }

    auto result = m_undoStack.back().cloud;
    m_undoStack.pop_back();

    qDebug() << "[CloudHistory] undo | undo depth:" << m_undoStack.size()
             << "redo depth:" << m_redoStack.size();
    emit historyChanged();
    return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudHistory::redo(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &currentCloud)
{
    if (m_redoStack.empty()) return nullptr;

    // Сохраняем текущее состояние в undo.
    if (currentCloud && !currentCloud->empty()) {
        Snapshot undoSnap;
        undoSnap.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*currentCloud);
        undoSnap.description = m_redoStack.back().description;
        m_undoStack.push_back(std::move(undoSnap));
    }

    auto result = m_redoStack.back().cloud;
    m_redoStack.pop_back();

    qDebug() << "[CloudHistory] redo | undo depth:" << m_undoStack.size()
             << "redo depth:" << m_redoStack.size();
    emit historyChanged();
    return result;
}

QString CloudHistory::undoDescription() const
{
    if (m_undoStack.empty()) return {};
    return m_undoStack.back().description;
}

QString CloudHistory::redoDescription() const
{
    if (m_redoStack.empty()) return {};
    return m_redoStack.back().description;
}

void CloudHistory::clear()
{
    m_undoStack.clear();
    m_redoStack.clear();
    qDebug() << "[CloudHistory] cleared";
    emit historyChanged();
}

void CloudHistory::setMaxDepth(int depth)
{
    m_maxDepth = qMax(1, depth);
    while (static_cast<int>(m_undoStack.size()) > m_maxDepth) {
        m_undoStack.pop_front();
    }
}

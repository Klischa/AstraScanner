#ifndef CLOUDHISTORY_H
#define CLOUDHISTORY_H

#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>

// Стек состояний облака точек для Undo/Redo.
//
// Перед каждой деструктивной операцией (SOR, ROR, VoxelGrid, MagicWand,
// загрузка скана, ICP-merge) вызывается pushState() — текущее облако
// копируется в undo-стек. Redo-стек очищается при новом pushState().
//
// Максимальная глубина стека ограничена (по умолчанию 20), чтобы не
// съедать память на больших облаках (200K точек × 20 = ~200 МБ).
class CloudHistory : public QObject
{
    Q_OBJECT
public:
    explicit CloudHistory(QObject *parent = nullptr);

    // Сохранить текущее состояние облака перед операцией.
    // Описание (description) — для отображения в UI ("SOR", "VoxelGrid" и т.п.).
    void pushState(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   const QString &description = {});

    // Отменить последнюю операцию. Возвращает облако из undo-стека
    // и перемещает текущее состояние в redo-стек.
    // Возвращает nullptr, если undo-стек пуст.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr undo(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &currentCloud);

    // Повторить отменённую операцию. Возвращает облако из redo-стека
    // и перемещает текущее состояние в undo-стек.
    // Возвращает nullptr, если redo-стек пуст.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr redo(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &currentCloud);

    bool canUndo() const { return !m_undoStack.empty(); }
    bool canRedo() const { return !m_redoStack.empty(); }

    int undoDepth() const { return static_cast<int>(m_undoStack.size()); }
    int redoDepth() const { return static_cast<int>(m_redoStack.size()); }

    // Описание последней операции в undo/redo стеке (для тултипов).
    QString undoDescription() const;
    QString redoDescription() const;

    // Очистить оба стека (например, при Clear или новом сканировании).
    void clear();

    // Максимальная глубина undo-стека.
    int maxDepth() const { return m_maxDepth; }
    void setMaxDepth(int depth);

signals:
    void historyChanged();  // Эмитится при pushState/undo/redo/clear

private:
    struct Snapshot {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        QString description;
    };

    std::deque<Snapshot> m_undoStack;
    std::deque<Snapshot> m_redoStack;
    int m_maxDepth = 20;
};

#endif // CLOUDHISTORY_H

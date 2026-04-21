#ifndef PROJECTMANAGER_H
#define PROJECTMANAGER_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QDateTime>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Одна единица скана внутри проекта: облако точек + метаданные.
struct ScanItem {
    QString name;                                           // пользовательское имя
    QString relativeFilePath;                               // относительный путь к .ply внутри директории проекта
    QDateTime createdAt;
    int pointCount = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;           // загружено в память (может быть null до load)
};

// Проект — это директория со следующим layout:
//   <projectDir>/
//     project.json          — метаданные, список сканов
//     scans/
//       scan_<timestamp>.ply
//       ...
//     mesh.ply              — (опционально) реконструированный меш
//
// ProjectManager владеет текущим проектом и списком сканов. Сохранение —
// сериализация в project.json + запись PLY для каждого облака.
class ProjectManager : public QObject
{
    Q_OBJECT
public:
    explicit ProjectManager(QObject *parent = nullptr);

    // --- Управление проектом ---
    bool newProject(const QString &projectDir, const QString &name = QString());
    bool openProject(const QString &projectDir);
    bool saveProject();
    bool saveProjectAs(const QString &projectDir);
    void closeProject();

    bool isOpen() const { return !m_projectDir.isEmpty(); }
    bool isDirty() const { return m_dirty; }
    QString projectDir() const { return m_projectDir; }
    QString projectName() const { return m_projectName; }

    // --- Сканы ---
    // Добавляет облако в проект как новый скан. Облако *копируется* (shallow),
    // затем сохраняется на диск в scans/ внутри директории проекта. Возвращает
    // индекс добавленного элемента или -1 при ошибке.
    int addScan(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                const QString &name = QString());

    bool removeScan(int index);
    bool renameScan(int index, const QString &newName);

    const QVector<ScanItem> &scans() const { return m_scans; }
    int scanCount() const { return m_scans.size(); }

    // Возвращает облако — при необходимости подгружает из файла.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanCloud(int index);

    QString lastError() const { return m_lastError; }

signals:
    void projectChanged();          // метаданные или состояние dirty
    void scansChanged();            // добавлен/удалён/переименован скан

private:
    QString metadataPath() const;
    QString scansDirPath() const;
    bool writeMetadata();
    bool readMetadata();
    bool saveScanToDisk(ScanItem &item);
    bool loadScanFromDisk(ScanItem &item);
    void setDirty(bool dirty);

    QString m_projectDir;
    QString m_projectName;
    QVector<ScanItem> m_scans;
    bool m_dirty = false;
    QString m_lastError;
};

#endif // PROJECTMANAGER_H

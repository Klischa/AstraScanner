#ifndef EXPORTMANAGER_H
#define EXPORTMANAGER_H

#include <QObject>
#include <QString>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

// Экспорт облаков точек (PLY) и полигональных мешей (STL/OBJ/PLY) на диск.
// Формат определяется по расширению файла; методы saveAs* позволяют явно
// выбрать формат.
//
// Возвращаемое значение bool — успех/провал. Текст ошибки можно получить
// через lastError(). Все методы thread-safe только в смысле «можно вызывать
// из одного потока»: pcl::io::* внутри не синхронизированы.
class ExportManager : public QObject
{
    Q_OBJECT
public:
    enum class CloudFormat { Auto, PLY, PCD };
    enum class MeshFormat  { Auto, PLY, STL, OBJ };

    explicit ExportManager(QObject *parent = nullptr);

    // Point cloud import (PLY / PCD). Формат определяется по расширению.
    // Возвращает nullptr при ошибке; текст — в lastError().
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCloud(const QString &filename);

    // Point cloud export.
    bool savePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                        const QString &filename,
                        CloudFormat format = CloudFormat::Auto);

    // Polygon mesh export (meш должен быть непустым).
    bool savePolygonMesh(const pcl::PolygonMesh &mesh,
                         const QString &filename,
                         MeshFormat format = MeshFormat::Auto);

    const QString &lastError() const { return m_lastError; }

    // Возвращает список пар (suffix, displayName) для QFileDialog фильтров.
    // Пример: ("ply", "Stanford Polygon Format (*.ply)").
    static QStringList cloudFileFilters();
    static QStringList meshFileFilters();

signals:
    void exportStarted(const QString &filename);
    void exportFinished(const QString &filename, bool success);

private:
    static CloudFormat detectCloudFormat(const QString &filename);
    static MeshFormat  detectMeshFormat(const QString &filename);

    QString m_lastError;
};

#endif // EXPORTMANAGER_H

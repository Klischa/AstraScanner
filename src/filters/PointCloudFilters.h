#ifndef POINTCLOUDFILTERS_H
#define POINTCLOUDFILTERS_H

#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

class PointCloudFilters : public QObject
{
    Q_OBJECT
public:
    // Параметры Poisson-реконструкции. Значения по умолчанию выбраны под
    // объекты 0.2–0.5 м, отсканированные Astra Pro на расстоянии ~0.5–1.0 м.
    // depth — логарифм размера сетки октодерева (8 = 256^3, 9 = 512^3, …).
    // С increase depth качество растёт, но время и память — экспоненциально.
    // pointWeight (screenedPoissonWeight) >0 включает screened-вариант:
    // восстановленная поверхность ближе следует исходным точкам.
    struct PoissonParams {
        int depth = 9;
        int minDepth = 5;
        float pointWeight = 4.0f;
        float samplesPerNode = 1.5f;
        float scale = 1.1f;
        bool confidence = false;
        bool outputPolygons = false;
        // Радиус поиска соседей для оценки нормалей; 0 → k-nearest вместо
        // radius search (см. kNearest).
        double normalSearchRadius = 0.01;
        int kNearest = 20;
    };

    explicit PointCloudFilters(QObject *parent = nullptr);

    // Фильтры очистки
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyStatisticalOutlierRemoval(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        int meanK = 50, double stddevMulThresh = 1.0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyRadiusOutlierRemoval(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        double radius = 0.02, int minNeighbors = 10);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyVoxelGrid(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        float leafSize = 0.005f);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyMagicWand(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        float leafSize = 0.005f, int meanK = 50, double stddevMulThresh = 1.0);

    // Регистрация (ICP)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registerPointCloudsICP(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
        double maxCorrespondenceDistance = 0.05,
        int maximumIterations = 50);

    // Объединение нескольких сканов: последовательная pairwise-регистрация
    // через ICP. scans[0] используется как «якорь» (reference), каждый
    // следующий скан выравнивается относительно уже накопленного облака.
    // При сбоях ICP на отдельном скане он либо добавляется «как есть»
    // (skipNonConverged=false), либо пропускается (true). Дополнительно
    // применяется финальная воксельная децимация, если voxelLeafOut > 0,
    // чтобы объединённое облако не раздувалось в разы.
    struct MergeParams {
        double maxCorrespondenceDistance = 0.05;   // 5 см
        int maximumIterations = 50;
        bool skipNonConverged = false;
        double voxelLeafOut = 0.0;                 // 0 → без децимации
    };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeScans(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &scans,
        const MergeParams &params = {});

    // Poisson Surface Reconstruction: строит замкнутый водонепроницаемый меш
    // из плотного облака точек. Внутри оценивает нормали (если их нет),
    // ориентирует их, затем запускает pcl::Poisson. Возвращает непустой
    // pcl::PolygonMesh при успехе; при ошибке mesh.polygons будет пустым и
    // сообщение уйдёт через qWarning. Может занимать секунды-минуты на
    // плотных облаках; вызывать из worker-потока (QtConcurrent::run и т.п.).
    pcl::PolygonMesh reconstructPoissonMesh(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const PoissonParams &params = {});

signals:
    void progressUpdated(int percentage);
    void filterCompleted(const QString &filterName, int pointsBefore, int pointsAfter);

private:
    void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals);
};

#endif // POINTCLOUDFILTERS_H
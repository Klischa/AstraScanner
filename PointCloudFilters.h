#ifndef POINTCLOUDFILTERS_H
#define POINTCLOUDFILTERS_H

#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

class PointCloudFilters : public QObject
{
    Q_OBJECT
public:
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

    // Объединение нескольких сканов
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeScans(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &scans);

signals:
    void progressUpdated(int percentage);
    void filterCompleted(const QString &filterName, int pointsBefore, int pointsAfter);

private:
    void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals);
};

#endif // POINTCLOUDFILTERS_H
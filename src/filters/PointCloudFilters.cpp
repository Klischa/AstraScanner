#include "PointCloudFilters.h"
#include <QDebug>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

PointCloudFilters::PointCloudFilters(QObject *parent) : QObject(parent) {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::applyStatisticalOutlierRemoval(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    int meanK, double stddevMulThresh)
{
    if (!cloud || cloud->empty()) return cloud;

    int pointsBefore = cloud->size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*filtered);

    int pointsAfter = filtered->size();
    emit filterCompleted("Statistical Outlier Removal", pointsBefore, pointsAfter);

    qInfo() << "SOR:" << pointsBefore << "->" << pointsAfter << "points";
    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::applyRadiusOutlierRemoval(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    double radius, int minNeighbors)
{
    if (!cloud || cloud->empty()) return cloud;

    int pointsBefore = cloud->size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(minNeighbors);
    ror.filter(*filtered);

    int pointsAfter = filtered->size();
    emit filterCompleted("Radius Outlier Removal", pointsBefore, pointsAfter);

    qInfo() << "ROR:" << pointsBefore << "->" << pointsAfter << "points";
    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::applyVoxelGrid(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    float leafSize)
{
    if (!cloud || cloud->empty()) return cloud;

    int pointsBefore = cloud->size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*filtered);

    int pointsAfter = filtered->size();
    emit filterCompleted("Voxel Grid", pointsBefore, pointsAfter);

    qInfo() << "Voxel:" << pointsBefore << "->" << pointsAfter << "points";
    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::applyMagicWand(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    float leafSize, int meanK, double stddevMulThresh)
{
    if (!cloud || cloud->empty()) return cloud;

    int pointsBefore = cloud->size();

    auto voxelized = applyVoxelGrid(cloud, leafSize);
    auto filtered = applyStatisticalOutlierRemoval(voxelized, meanK, stddevMulThresh);

    int pointsAfter = filtered->size();
    emit filterCompleted("Magic Wand", pointsBefore, pointsAfter);

    qInfo() << "Magic Wand:" << pointsBefore << "->" << pointsAfter << "points";
    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::registerPointCloudsICP(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
    double maxCorrespondenceDistance,
    int maximumIterations)
{
    if (!source || source->empty() || !target || target->empty()) {
        return source;
    }

    emit progressUpdated(10);

    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);

    computeNormals(source, sourceNormals);
    computeNormals(target, targetNormals);

    emit progressUpdated(30);

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setMaximumIterations(maximumIterations);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-8);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
    icp.align(*aligned);

    emit progressUpdated(80);

    if (icp.hasConverged()) {
        qInfo() << "ICP converged with score:" << icp.getFitnessScore();
        emit progressUpdated(100);
        return aligned;
    } else {
        qWarning() << "ICP did not converge";
        emit progressUpdated(100);
        return source;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::mergeScans(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &scans)
{
    if (scans.empty()) return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>);
    *merged = *scans[0];

    for (size_t i = 1; i < scans.size(); ++i) {
        emit progressUpdated(static_cast<int>((i * 100) / scans.size()));

        if (!scans[i] || scans[i]->empty()) continue;

        auto aligned = registerPointCloudsICP(scans[i], merged);
        *merged += *aligned;
    }

    emit progressUpdated(100);
    qInfo() << "Merged" << scans.size() << "scans into" << merged->size() << "points";
    return merged;
}

void PointCloudFilters::computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                      pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);
}
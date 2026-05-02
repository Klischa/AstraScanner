#include "PointCloudFilters.h"
#include <QDebug>
#include <QTimer>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Core>
#include <queue>
#include <vector>
#include <cmath>

// AI включения
#include "../ai/AiClient.h"

// Глобальный AI клиент для коммуникации с AIService
static AiClient *g_aiClient = nullptr;

void PointCloudFilters::setAiClient(AiClient *client)
{
    g_aiClient = client;
}

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
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &scans,
    const MergeParams &params)
{
    if (scans.empty()) return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Первый скан — опорный, выравнивать его не к чему.
    if (scans[0] && !scans[0]->empty()) *merged = *scans[0];

    int converged = 0;
    int skipped = 0;
    int addedAsIs = 0;

    for (size_t i = 1; i < scans.size(); ++i) {
        emit progressUpdated(static_cast<int>((i * 100) / scans.size()));

        const auto &src = scans[i];
        if (!src || src->empty() || merged->empty()) continue;

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(src);
        icp.setInputTarget(merged);
        icp.setMaxCorrespondenceDistance(params.maxCorrespondenceDistance);
        icp.setMaximumIterations(params.maximumIterations);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-8);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
        icp.align(*aligned);

        if (icp.hasConverged()) {
            *merged += *aligned;
            ++converged;
            qInfo() << "[mergeScans] scan" << i << "converged, score"
                    << icp.getFitnessScore();
        } else if (params.skipNonConverged) {
            ++skipped;
            qWarning() << "[mergeScans] scan" << i << "ICP did not converge — skipped";
        } else {
            // ICP не сошёлся — добавляем исходное облако без трансформации.
            // Это даст хоть какой-то результат, но ожидаемо будет double-wall.
            *merged += *src;
            ++addedAsIs;
            qWarning() << "[mergeScans] scan" << i
                       << "ICP did not converge — added as-is (expect misalignment)";
        }
    }

    if (params.voxelLeafOut > 0.0) {
        merged = applyVoxelGrid(merged, static_cast<float>(params.voxelLeafOut));
    }

    emit progressUpdated(100);
    qInfo() << "Merged" << scans.size() << "scans into" << merged->size()
            << "points (converged=" << converged
            << ", skipped=" << skipped
            << ", added-as-is=" << addedAsIs << ")";
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

pcl::PolygonMesh PointCloudFilters::reconstructPoissonMesh(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const PoissonParams &params)
{
    pcl::PolygonMesh mesh;
    if (!cloud || cloud->empty()) {
        qWarning() << "[Poisson] Empty input cloud";
        return mesh;
    }

    emit progressUpdated(5);

    // 1. Оценка нормалей. Используем OMP-версию — на 4+ ядрах это в разы
    //    быстрее, чем однопоточный NormalEstimation.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    if (params.normalSearchRadius > 0.0) {
        ne.setRadiusSearch(params.normalSearchRadius);
    } else {
        ne.setKSearch(params.kNearest);
    }
    // View-point для ориентации нормалей. Если пользователь задал custom
    // viewpoint — используем его как есть (например, чтобы указать «наружу»
    // относительно замкнутой фигуры). Иначе — эвристика: centroid облака,
    // смещённый на 1 м «к камере» вдоль Z.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    float vpX = centroid[0];
    float vpY = centroid[1];
    float vpZ = centroid[2] - 1.0f;
    if (params.useCustomViewpoint) {
        vpX = params.viewpointX;
        vpY = params.viewpointY;
        vpZ = params.viewpointZ;
        qInfo() << "[Poisson] Using custom viewpoint ("
                << vpX << vpY << vpZ << ")";
    }
    ne.setViewPoint(vpX, vpY, vpZ);
    ne.compute(*normals);

    if (normals->size() != cloud->size()) {
        qWarning() << "[Poisson] Normal estimation failed: got" << normals->size()
                   << "normals for" << cloud->size() << "points";
        return mesh;
    }
    emit progressUpdated(25);

    // Опциональная согласованная ориентация нормалей: BFS по k-nearest от
    // сида, ближайшего к view-point. Разворачиваем нормали соседей так,
    // чтобы dot(n_parent, n_child) ≥ 0. Сид-точку ориентируем «наружу»
    // относительно view-point, то есть нормалью в сторону view-point.
    if (params.consistentOrientation) {
        const int k = std::max(3, params.orientationKNeighbors);
        pcl::KdTreeFLANN<pcl::PointXYZRGB> orientTree;
        orientTree.setInputCloud(cloud);

        // Ищем ближайшую к view-point точку в облаке — это сид.
        pcl::PointXYZRGB vpPoint;
        vpPoint.x = vpX; vpPoint.y = vpY; vpPoint.z = vpZ;
        std::vector<int> seedIdx(1);
        std::vector<float> seedDist(1);
        if (orientTree.nearestKSearch(vpPoint, 1, seedIdx, seedDist) < 1) {
            qWarning() << "[Poisson] Consistent orientation: seed search failed";
        } else {
            const int seed = seedIdx[0];

            // Ориентируем сид нормалью «в сторону view-point».
            Eigen::Vector3f toVp(
                vpX - cloud->points[seed].x,
                vpY - cloud->points[seed].y,
                vpZ - cloud->points[seed].z);
            Eigen::Vector3f nSeed(
                normals->points[seed].normal_x,
                normals->points[seed].normal_y,
                normals->points[seed].normal_z);
            if (toVp.norm() > 1e-6f && nSeed.dot(toVp) < 0.0f) {
                normals->points[seed].normal_x *= -1.0f;
                normals->points[seed].normal_y *= -1.0f;
                normals->points[seed].normal_z *= -1.0f;
            }

            std::vector<bool> visited(cloud->size(), false);
            std::queue<int> bfs;
            bfs.push(seed);
            visited[seed] = true;

            std::vector<int> nbIdx(k);
            std::vector<float> nbDist(k);

            std::size_t processed = 0;
            const std::size_t total = cloud->size();
            while (!bfs.empty()) {
                int idx = bfs.front(); bfs.pop();
                ++processed;

                int found = orientTree.nearestKSearch(
                    cloud->points[idx], k, nbIdx, nbDist);
                if (found <= 0) continue;

                const Eigen::Vector3f parent(
                    normals->points[idx].normal_x,
                    normals->points[idx].normal_y,
                    normals->points[idx].normal_z);

                for (int i = 0; i < found; ++i) {
                    const int n = nbIdx[i];
                    if (n < 0 || static_cast<std::size_t>(n) >= cloud->size()) continue;
                    if (visited[n]) continue;
                    visited[n] = true;

                    Eigen::Vector3f child(
                        normals->points[n].normal_x,
                        normals->points[n].normal_y,
                        normals->points[n].normal_z);
                    if (parent.dot(child) < 0.0f) {
                        normals->points[n].normal_x *= -1.0f;
                        normals->points[n].normal_y *= -1.0f;
                        normals->points[n].normal_z *= -1.0f;
                    }
                    bfs.push(n);
                }

                // Периодический прогресс (25 → 35) — BFS обычно занимает
                // заметное время на больших облаках.
                if ((processed & 0x3FFF) == 0 && total > 0) {
                    int pct = 25 + static_cast<int>(10 * processed / total);
                    emit progressUpdated(std::min(35, pct));
                }
            }
            qInfo() << "[Poisson] Consistent orientation propagated from seed"
                    << seed << "over" << processed << "points";
        }
    }

    // Явная инверсия всех нормалей — последний шаг, применяется поверх
    // consistent-orientation, если Poisson всё равно строит меш «наизнанку».
    if (params.flipNormals) {
        for (auto &n : normals->points) {
            n.normal_x *= -1.0f;
            n.normal_y *= -1.0f;
            n.normal_z *= -1.0f;
        }
        qInfo() << "[Poisson] Normals inverted (flipNormals=true)";
    }
    emit progressUpdated(35);

    // 2. Склеиваем XYZRGB + Normal → PointNormal (Poisson в PCL ожидает его).
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
    cloudWithNormals->reserve(cloud->size());
    for (std::size_t i = 0; i < cloud->size(); ++i) {
        pcl::PointNormal p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        p.normal_x = normals->points[i].normal_x;
        p.normal_y = normals->points[i].normal_y;
        p.normal_z = normals->points[i].normal_z;
        p.curvature = normals->points[i].curvature;
        if (!std::isfinite(p.x) || !std::isfinite(p.normal_x)) continue;
        cloudWithNormals->push_back(p);
    }
    cloudWithNormals->width = cloudWithNormals->size();
    cloudWithNormals->height = 1;
    cloudWithNormals->is_dense = true;

    if (cloudWithNormals->empty()) {
        qWarning() << "[Poisson] No finite oriented points after filtering";
        return mesh;
    }
    emit progressUpdated(45);

    // 3. Собственно Poisson. Параметры screened-варианта дают меньше
    //    «раздутия» меша; при этом depth ≥ 8 критичен для тонких деталей.
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(cloudWithNormals);
    poisson.setDepth(params.depth);
    poisson.setMinDepth(params.minDepth);
    poisson.setPointWeight(params.pointWeight);
    poisson.setSamplesPerNode(params.samplesPerNode);
    poisson.setScale(params.scale);
    poisson.setConfidence(params.confidence);
    poisson.setOutputPolygons(params.outputPolygons);
    poisson.reconstruct(mesh);

    emit progressUpdated(100);

    const std::size_t nPoly = mesh.polygons.size();
    if (nPoly == 0) {
        qWarning() << "[Poisson] Reconstruction returned empty mesh";
    } else {
        qInfo() << "[Poisson] Reconstructed" << nPoly << "polygons from"
                << cloudWithNormals->size() << "oriented points"
                << "(depth=" << params.depth << ")";
    }
    return mesh;
}

// === AI Сегментация NPMFF-Net ===

PointCloudFilters::SegmentationResult PointCloudFilters::segmentNPMFF(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const NPMFFParams &params)
{
    SegmentationResult result;
    result.success = false;
    
    if (!cloud || cloud->empty()) {
        result.error = "Empty point cloud";
        return result;
    }
    
    // Если AI клиент доступен - используем его
    if (g_aiClient && g_aiClient->isAvailable()) {
        qInfo() << "[NPMFF] Starting AI segmentation, cloud size:" << cloud->size();

        QEventLoop loop;
        QVector<int> receivedIndices;
        QTimer timeoutTimer;
        timeoutTimer.setSingleShot(true);
        timeoutTimer.start(30000);

        QObject::connect(g_aiClient, &AiClient::segmentationFinished,
                        [&](const QVector<int> &indices, bool success) {
            receivedIndices = indices;
            result.success = success;
            qInfo() << "[NPMFF] AI response: success=" << success << ", points=" << indices.size();
            loop.quit();
        });

        QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
            qWarning() << "[NPMFF] Timeout after 30s";
            result.error = "timeout";
            result.success = false;
            loop.quit();
        });

        qInfo() << "[NPMFF] Calling AI service...";
        g_aiClient->segmentNPMFF(cloud);
        qInfo() << "[NPMFF] Waiting...";
        loop.exec();
        qInfo() << "[NPMFF] Done, success=" << result.success;

        if (result.success) {
            result.foregroundIndices = receivedIndices;
            qInfo() << "[NPMFF] AI segmentation succeeded with" << receivedIndices.size() << "points";
        } else {
            result.error = g_aiClient->lastError();
            qWarning() << "[NPMFF] AI segmentation failed:" << result.error;
        }

        return result;
    }
    
    // Fallback: используем встроенные алгоритмы сегментации (Region Growing)
    try {
        emit progressUpdated(20);
        
        // Оценка нормалей
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(20);
        ne.compute(*normals);
        
        emit progressUpdated(40);
        
        // Region Growing сегментация
        pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setMinClusterSize(params.minClusterSize);
        reg.setSmoothnessThreshold(params.smoothnessWeight * M_PI / 180.0);
        reg.setCurvatureThreshold(1.0);
        reg.setResidualThreshold(params.densityThreshold);
        
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        
        emit progressUpdated(80);
        
        // Находим самый большой кластер
        int maxClusterIdx = 0;
        std::size_t maxClusterSize = 0;
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            if (clusters[i].indices.size() > maxClusterSize) {
                maxClusterSize = clusters[i].indices.size();
                maxClusterIdx = static_cast<int>(i);
            }
        }

        if (!clusters.empty() && maxClusterSize > 0) {
            // Convert PCL indices to QVector
            QVector<int> indices;
            for (int idx : clusters[maxClusterIdx].indices) indices.append(idx);
            result.foregroundIndices = indices;
            result.success = true;
            qInfo() << "[NPMFF] Region Growing found" << clusters.size()
                   << "clusters, largest has" << maxClusterSize << "points";
        }

        emit progressUpdated(100);
        
    } catch (const std::exception &e) {
        result.error = e.what();
        qWarning() << "[NPMFF] Segmentation error:" << e.what();
    }
    
    return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::filterByIndices(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const QVector<int> &indices,
    bool keepIndices)
{
    if (!cloud || cloud->empty()) return cloud;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (keepIndices) {
        // Сохранить только указанные индексы
        for (int idx : indices) {
            if (idx >= 0 && idx < static_cast<int>(cloud->size())) {
                filtered->push_back(cloud->points[idx]);
            }
        }
    } else {
        // Удалить указанные индексы
        std::set<int> indexSet = std::set<int>(indices.begin(), indices.end());
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            if (indexSet.find(i) == indexSet.end()) {
                filtered->push_back(cloud->points[i]);
            }
        }
    }
    
    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = true;
    
    int pointsBefore = cloud->size();
    int pointsAfter = filtered->size();
    emit filterCompleted("FilterByIndices", pointsBefore, pointsAfter);
    
    qInfo() << "FilterByIndices:" << pointsBefore << "->" << pointsAfter << "points";
    return filtered;
}

// === Регистрация BUFFER-X ===

PointCloudFilters::RegistrationResult PointCloudFilters::registerBufferX(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
    const BufferXParams &params)
{
    RegistrationResult result;
    result.transformation = Eigen::Matrix4f::Identity();
    result.success = false;
    
    if (!source || source->empty() || !target || target->empty()) {
        result.error = "Empty point cloud";
        return result;
    }
    
    // Если AI клиент доступен - используем его
    if (g_aiClient && g_aiClient->isAvailable()) {
        qInfo() << "[BUFFER-X] Starting registration...";
        QEventLoop loop;
        QTimer timeoutTimer;
        timeoutTimer.setSingleShot(true);
        timeoutTimer.start(60000); // 60s timeout

        QObject::connect(g_aiClient, &AiClient::registrationFinished,
                        [&](const Eigen::Matrix4f &transform, bool success) {
            result.transformation = transform;
            result.success = success;
            result.fitness = success ? 0.95f : 0.0f;
            qInfo() << "[BUFFER-X] AI response: success=" << success;
            loop.quit();
        });

        QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
            qWarning() << "[BUFFER-X] Timeout after 60s";
            result.error = "timeout";
            result.success = false;
            loop.quit();
        });

        g_aiClient->registerBUFFERX(source, target, params.useICPRefinement);
        loop.exec();

        if (!result.success) {
            result.error = g_aiClient->lastError();
            qWarning() << "[BUFFER-X] Failed:" << result.error;
        }

        // Если включен ICP refinement
        if (result.success && params.useICPRefinement) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*source, *aligned, result.transformation);
            
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setInputSource(aligned);
            icp.setInputTarget(target);
            icp.setMaxCorrespondenceDistance(params.icpMaxDistance);
            icp.setMaximumIterations(params.icpMaxIterations);
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpResult(new pcl::PointCloud<pcl::PointXYZRGB>);
            icp.align(*icpResult);
            
            result.icpTransformation = icp.getFinalTransformation();
            result.converged = icp.hasConverged();
            result.fitness = icp.getFitnessScore();
            
            // Комбинируем трансформации
            result.transformation = result.icpTransformation * result.transformation;
        }
        
        return result;
    }
    
    // Fallback: просто ICP
    qInfo() << "[BUFFER-X] Using fallback ICP registration";
    return registerHybrid(source, target, params);
}

PointCloudFilters::RegistrationResult PointCloudFilters::registerHybrid(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
    const BufferXParams &params)
{
    RegistrationResult result;
    result.transformation = Eigen::Matrix4f::Identity();
    result.success = false;
    
    if (!source || source->empty() || !target || target->empty()) {
        result.error = "Empty point cloud";
        return result;
    }
    
    emit progressUpdated(10);
    
    // Простой ICP для грубой регистрации
    auto aligned = registerPointCloudsICP(source, target,
                                     params.icpMaxDistance,
                                     params.icpMaxIterations);
    
    emit progressUpdated(80);
    
    // Вычисляем трансформацию
    Eigen::Vector4f centroid1, centroid2;
    pcl::compute3DCentroid(*source, centroid1);
    pcl::compute3DCentroid(*target, centroid2);
    
    result.transformation.block<3, 1>(0, 3) = centroid2.block<3, 1>(0, 0) - centroid1.block<3, 1>(0, 0);
    
    // ICP финальная подгонка если включена
    if (params.useICPRefinement) {
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(aligned);
        icp.setInputTarget(target);
        icp.setMaxCorrespondenceDistance(params.icpMaxDistance);
        icp.setMaximumIterations(params.icpMaxIterations);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpResult(new pcl::PointCloud<pcl::PointXYZRGB>);
        icp.align(*icpResult);
        
        result.icpTransformation = icp.getFinalTransformation();
        result.converged = icp.hasConverged();
        result.fitness = icp.getFitnessScore();
    }
    
    result.success = true;
    emit progressUpdated(100);
    
    qInfo() << "[BUFFER-X+ICP] Registration completed, fitness:" << result.fitness;
    return result;
}

PointCloudFilters::RegistrationResult PointCloudFilters::registerDINO(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target,
    bool useColorImages)
{
    RegistrationResult result;
    result.transformation = Eigen::Matrix4f::Identity();
    result.success = false;
    
    if (!source || source->empty() || !target || target->empty()) {
        result.error = "Empty point cloud";
        return result;
    }
    
    // Если AI клиент доступен - используем его
    if (g_aiClient && g_aiClient->isAvailable()) {
        QEventLoop loop;
        
        QObject::connect(g_aiClient, &AiClient::registrationFinished,
                        [&](const Eigen::Matrix4f &transform, bool success) {
            result.transformation = transform;
            result.success = success;
            result.fitness = success ? 0.9f : 0.0f;
            loop.quit();
        });
        
        g_aiClient->registerDINO(source, target, QByteArray(), QByteArray());
        loop.exec();
        
        if (!result.success) {
            result.error = g_aiClient->lastError();
        }
        
        return result;
    }
    
    // Fallback: используем ICP
    qInfo() << "[DINOReg] Using fallback ICP registration";
    return registerHybrid(source, target, BufferXParams());
}

// === Легкая Mesh генерация ===

PointCloudFilters::MeshResult PointCloudFilters::generateLightweightMesh(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const MeshQualityParams &params)
{
    MeshResult result;
    result.success = false;
    
    if (!cloud || cloud->empty()) {
        result.error = "Empty point cloud";
        return result;
    }
    
    // Если AI клиент доступен - используем его
    if (g_aiClient && g_aiClient->isAvailable()) {
        QEventLoop loop;
        QString meshPath;
        
        QObject::connect(g_aiClient, &AiClient::meshGenerationFinished,
                        [&](const QString &path, bool success) {
            meshPath = path;
            result.success = success;
            result.success = success;
            loop.quit();
        });
        
        AiClient::MeshQuality quality;
        switch (params.quality) {
            case MeshQualityParams::Quality::Low:
                quality = AiClient::MeshQuality::Low; break;
            case MeshQualityParams::Quality::High:
                quality = AiClient::MeshQuality::High; break;
            default:
                quality = AiClient::MeshQuality::Medium;
        }
        
        g_aiClient->generateMeshLightweight(cloud, quality, "ply");
        loop.exec();
        
        if (result.success) {
            // Загружаем mesh из файла
            pcl::io::loadPLYFile(meshPath.toStdString(), result.mesh);
            result.vertexCount = result.mesh.cloud.width;
            result.faceCount = result.mesh.polygons.size();
        } else {
            result.error = g_aiClient->lastError();
        }
        
        return result;
    }
    
    // Fallback: используем Poisson с пониженным качеством
    try {
        emit progressUpdated(20);
        
        PoissonParams poissonParams;
        switch (params.quality) {
            case MeshQualityParams::Quality::Low:
                poissonParams.depth = 6;
                break;
            case MeshQualityParams::Quality::High:
                poissonParams.depth = 10;
                break;
            default:
                poissonParams.depth = 8;
        }
        
        result.mesh = reconstructPoissonMesh(cloud, poissonParams);
        
        result.vertexCount = result.mesh.cloud.width;
        result.faceCount = result.mesh.polygons.size();
        result.success = result.faceCount > 0;
        
        if (!result.success) {
            result.error = "Mesh reconstruction failed";
        }
        
        emit progressUpdated(100);
        qInfo() << "[LightweightMR] Generated" << result.faceCount << "faces";
        
    } catch (const std::exception &e) {
        result.error = e.what();
        qWarning() << "[LightweightMR] Error:" << e.what();
    }
    
    return result;
}

// === SuperPC и RARE ===

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::enhanceSuperPC(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const SuperPCParams &params)
{
    if (!cloud || cloud->empty()) return cloud;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Если AI клиент доступен - используем его
    if (g_aiClient && g_aiClient->isAvailable()) {
        QEventLoop loop;
        QString resultPath;
        
        QObject::connect(g_aiClient, &AiClient::enhancementFinished,
                        [&](const QString &path, bool success) {
            if (success) resultPath = path;
            loop.quit();
        });
        
        QStringList operations;
        if (params.denoise) operations << "denoise";
        if (params.fill) operations << "fill";
        if (params.densify) operations << "densify";
        if (params.colorize) operations << "colorize";
        
        g_aiClient->enhanceSuperPC(cloud, operations);
        loop.exec();
        
        if (!resultPath.isEmpty()) {
            pcl::io::loadPLYFile(resultPath.toStdString(), *resultCloud);
            return resultCloud;
        }
    }
    
    // Fallback: применяем локальные фильтры
    emit progressUpdated(20);
    
    if (params.denoise) {
        resultCloud = applyStatisticalOutlierRemoval(cloud, 50, 1.0);
    } else {
        resultCloud = cloud;
    }
    
    emit progressUpdated(50);
    
    if (params.densify) {
        // Простое сглаживание
        resultCloud = applyVoxelGrid(resultCloud, 0.005f);
    }
    
    emit progressUpdated(100);
    
    int pointsBefore = cloud->size();
    int pointsAfter = resultCloud->size();
    emit filterCompleted("SuperPC", pointsBefore, pointsAfter);
    
    qInfo() << "SuperPC:" << pointsBefore << "->" << pointsAfter;
    return resultCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFilters::refineRARE(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    float qualityImprovement)
{
    if (!cloud || cloud->empty()) return cloud;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Если AI клиент доступен - используем его
    if (g_aiClient && g_aiClient->isAvailable()) {
        QEventLoop loop;
        QString resultPath;
        
        QObject::connect(g_aiClient, &AiClient::refinementFinished,
                        [&](const QString &path, bool success) {
            if (success) resultPath = path;
            loop.quit();
        });
        
        g_aiClient->refineRARE(cloud);
        loop.exec();
        
        if (!resultPath.isEmpty()) {
            pcl::io::loadPLYFile(resultPath.toStdString(), *resultCloud);
            return resultCloud;
        }
    }
    
    // Fallback: легкое сглаживание
    emit progressUpdated(50);
    
    resultCloud = applyStatisticalOutlierRemoval(cloud, 30, 0.8);
    
    emit progressUpdated(100);
    
    qInfo() << "RARE refined:" << cloud->size() << "->" << resultCloud->size();
    return resultCloud;
}
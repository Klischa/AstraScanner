#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/stl_io.h>

int main() {
    std::cout << "=== AstraScanner Test Suite ===" << std::endl;

    // Создаем тестовое облако точек
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = static_cast<float>(rand()) / RAND_MAX * 2.0f - 1.0f;
        cloud->points[i].y = static_cast<float>(rand()) / RAND_MAX * 2.0f - 1.0f;
        cloud->points[i].z = static_cast<float>(rand()) / RAND_MAX * 2.0f - 1.0f;
        cloud->points[i].r = rand() % 256;
        cloud->points[i].g = rand() % 256;
        cloud->points[i].b = rand() % 256;
    }

    std::cout << "Created test cloud with " << cloud->size() << " points" << std::endl;

    // Тест фильтров
    std::cout << "\n=== Testing Filters ===" << std::endl;

    // SOR фильтр
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);

    std::cout << "SOR: " << cloud->size() << " -> " << cloud_filtered->size() << " points" << std::endl;

    // Воксельный фильтр
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud_filtered);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel.filter(*cloud_voxel);

    std::cout << "Voxel: " << cloud_filtered->size() << " -> " << cloud_voxel->size() << " points" << std::endl;

    // Тест экспорта
    std::cout << "\n=== Testing Export ===" << std::endl;

    // PLY экспорт
    if (pcl::io::savePLYFileBinary("test_cloud.ply", *cloud_voxel) == 0) {
        std::cout << "PLY export: SUCCESS" << std::endl;
    } else {
        std::cout << "PLY export: FAILED" << std::endl;
    }

    // Создание mesh через Poisson
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setInputCloud(cloud_voxel);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud_voxel, *normals, *cloud_with_normals);

    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDepth(8);
    poisson.setInputCloud(cloud_with_normals);

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    poisson.reconstruct(*mesh);

    // STL экспорт
    if (pcl::io::savePolygonFileSTL("test_mesh.stl", *mesh) == 0) {
        std::cout << "STL export: SUCCESS" << std::endl;
    } else {
        std::cout << "STL export: FAILED" << std::endl;
    }

    // OBJ экспорт
    if (pcl::io::saveOBJFile("test_mesh.obj", *mesh) == 0) {
        std::cout << "OBJ export: SUCCESS" << std::endl;
    } else {
        std::cout << "OBJ export: FAILED" << std::endl;
    }

    // Тест ICP
    std::cout << "\n=== Testing ICP Registration ===" << std::endl;

    // Создаем два похожих облака
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);

    *source = *cloud_voxel;
    *target = *cloud_voxel;

    // Сдвигаем source
    for (auto &point : source->points) {
        point.x += 0.1f;
        point.y += 0.05f;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(50);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
    icp.align(*aligned);

    if (icp.hasConverged()) {
        std::cout << "ICP: CONVERGED with fitness score: " << icp.getFitnessScore() << std::endl;
    } else {
        std::cout << "ICP: FAILED to converge" << std::endl;
    }

    std::cout << "\n=== Test Suite Complete ===" << std::endl;
    return 0;
}
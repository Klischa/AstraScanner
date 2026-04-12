#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/stl_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

int main() {
    std::cout << "=== AstraScanner Export Test ===" << std::endl;

    // Создаем тестовое облако точек (куб)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (float x = -0.5f; x <= 0.5f; x += 0.01f) {
        for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
            for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
                if (x*x + y*y + z*z <= 0.25f) { // Сфера внутри куба
                    pcl::PointXYZRGB point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                    cloud->points.push_back(point);
                }
            }
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    std::cout << "Created test sphere cloud with " << cloud->size() << " points" << std::endl;

    // Экспорт в PLY
    if (pcl::io::savePLYFileBinary("test_sphere.ply", *cloud) == 0) {
        std::cout << "✅ PLY export: SUCCESS" << std::endl;
    } else {
        std::cout << "❌ PLY export: FAILED" << std::endl;
    }

    // Создание mesh через Poisson
    std::cout << "Creating mesh via Poisson reconstruction..." << std::endl;

    // Вычисляем нормали
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // Объединяем точки и нормали
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Poisson reconstruction
    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDepth(8);
    poisson.setInputCloud(cloud_with_normals);

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    poisson.reconstruct(*mesh);

    std::cout << "Mesh created with " << mesh->polygons.size() << " polygons" << std::endl;

    // Экспорт в STL
    if (pcl::io::savePolygonFileSTL("test_sphere.stl", *mesh) == 0) {
        std::cout << "✅ STL export: SUCCESS" << std::endl;
    } else {
        std::cout << "❌ STL export: FAILED" << std::endl;
    }

    // Экспорт в OBJ
    if (pcl::io::saveOBJFile("test_sphere.obj", *mesh) == 0) {
        std::cout << "✅ OBJ export: SUCCESS" << std::endl;
    } else {
        std::cout << "❌ OBJ export: FAILED" << std::endl;
    }

    std::cout << "\n=== Export Test Complete ===" << std::endl;
    std::cout << "Check files: test_sphere.ply, test_sphere.stl, test_sphere.obj" << std::endl;

    return 0;
}
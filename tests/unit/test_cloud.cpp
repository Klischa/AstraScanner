#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxels.h>
#include <cmath>

using pcl::PointXYZRGB;
using CloudPtr = pcl::PointCloud<PointXYZRGB>::Ptr;

// ========== Тест: Создание облака точек ==========
TEST(CloudTest, CreatePointCloud)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    EXPECT_TRUE(cloud->empty());
    EXPECT_EQ(cloud->size(), 0u);
    
    // Добавляем точку
    PointXYZRGB pt;
    pt.x = 1.0f; pt.y = 2.0f; pt.z = 3.0f;
    pt.r = 255; pt.g = 0; pt.b = 0;
    cloud->push_back(pt);
    
    EXPECT_EQ(cloud->size(), 1u);
    EXPECT_FLOAT_EQ(cloud->front().x, 1.0f);
    EXPECT_FLOAT_EQ(cloud->front().y, 2.0f);
    EXPECT_FLOAT_EQ(cloud->front().z, 3.0f);
}

// ========== Тест: Сохранение в PCD формат ==========
TEST(CloudTest, SaveToPCD)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Создаем тестовое облако
    for (int i = 0; i < 100; ++i) {
        PointXYZRGB pt;
        pt.x = i * 0.01f;
        pt.y = i * 0.01f;
        pt.z = 0.0f;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    std::string filename = "test_cloud.pcd";
    
    // Сохраняем
    int res = pcl::io::savePCDFileBinary(filename, *cloud);
    EXPECT_EQ(res, 0);
    
    // Загружаем обратно
    CloudPtr loaded(new pcl::PointCloud<PointXYZRGB>);
    res = pcl::io::loadPCDFile(filename, *loaded);
    
    EXPECT_EQ(res, 0);
    EXPECT_EQ(loaded->size(), cloud->size());
    
    // Удаляем файл
    std::remove(filename.c_str());
}

// ========== Тест: Сохранение в PLY формат ==========
TEST(CloudTest, SaveToPLY)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Создаем тестовое облако
    for (int i = 0; i < 100; ++i) {
        PointXYZRGB pt;
        pt.x = i * 0.01f;
        pt.y = i * 0.01f;
        pt.z = i * 0.01f;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    std::string filename = "test_cloud.ply";
    
    // Сохраняем
    pcl::io::savePLYFile(filename, *cloud);
    
    // Проверяем файл создан
    EXPECT_TRUE(std::ifstream(filename).good());
    
    // Удаляем
    std::remove(filename.c_str());
}

// ========== Тест: Объединение облаков ==========
TEST(CloudTest, MergeClouds)
{
    CloudPtr cloud1(new pcl::PointCloud<PointXYZRGB>);
    CloudPtr cloud2(new pcl::PointCloud<PointXYZRGB>);
    
    // Заполняем cloud1
    for (int i = 0; i < 50; ++i) {
        PointXYZRGB pt;
        pt.x = i * 0.1f; pt.y = 0; pt.z = 0;
        pt.r = 255; pt.g = 0; pt.b = 0;
        cloud1->push_back(pt);
    }
    
    // Заполняем cloud2
    for (int i = 0; i < 50; ++i) {
        PointXYZRGB pt;
        pt.x = i * 0.1f; pt.y = 1; pt.z = 0;
        pt.r = 0; pt.g = 255; pt.b = 0;
        cloud2->push_back(pt);
    }
    
    // Объединяем
    *cloud1 += *cloud2;
    
    EXPECT_EQ(cloud1->size(), 100u);
}

// ========== Тест: Empty cloud merge ==========
TEST(CloudTest, EmptyCloudMerge)
{
    CloudPtr empty(new pcl::PointCloud<PointXYZRGB>);
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    PointXYZRGB pt;
    pt.x = 1; pt.y = 1; pt.z = 1;
    pt.r = 255; pt.g = 255; pt.b = 255;
    cloud->push_back(pt);
    
    // Пустое + непустое
    *empty += *cloud;
    EXPECT_EQ(empty->size(), 1u);
    
    // Непустое + пустое
    CloudPtrcloud2(new pcl::PointCloud<PointXYZRGB>);
    *cloud += *empty;
    EXPECT_EQ(cloud->size(), 1u);
}

// ========== Тест: Bounds calculation ==========
TEST(CloudTest, ComputeBounds)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Создаем облако с известными границами
    PointXYZRGB pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    pt.r = 255; pt.g = 255; pt.b = 255;
    cloud->push_back(pt);
    pt.x = 1; pt.y = 2; pt.z = 3;
    cloud->push_back(pt);
    pt.x = -1; pt.y = -2; pt.z = -3;
    cloud->push_back(pt);
    
    Eigen::Vector4f min_pt, max_pt;
    cloud->getMatrixBox().getBounds(min_pt, max_pt);
    
    EXPECT_FLOAT_EQ(min_pt[0], -1.0f);
    EXPECT_FLOAT_EQ(min_pt[1], -2.0f);
    EXPECT_FLOAT_EQ(min_pt[2], -3.0f);
    EXPECT_FLOAT_EQ(max_pt[0], 1.0f);
    EXPECT_FLOAT_EQ(max_pt[1], 2.0f);
    EXPECT_FLOAT_EQ(max_pt[2], 3.0f);
}

// ========== Тест: Инвалидные точки ==========
TEST(CloudTest, InvalidPoints)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Добавляем валидную точку
    PointXYZRGB valid;
    valid.x = 1; valid.y = 1; valid.z = 1;
    valid.r = 255; valid.g = 255; valid.b = 255;
    cloud->push_back(valid);
    
    // Добавляем невалидную точку
    PointXYZRGB invalid;
    invalid.x = std::nan("");
    invalid.y = std::nan("");
    invalid.z = std::nan("");
    invalid.r = 255; invalid.g = 255; invalid.b = 255;
    cloud->push_back(invalid);
    
    // Проверяем на NaN
    int validCount = 0;
    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            validCount++;
        }
    }
    
    EXPECT_EQ(validCount, 1);
}
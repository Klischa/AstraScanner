#include <gtest/gtest.h>
#include "PointCloudFilters.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cmath>

using pcl::PointXYZRGB;
using CloudPtr = pcl::PointCloud<PointXYZRGB>::Ptr;

// ========== Тест: Воксельный фильтр ==========
TEST(FiltersTest, VoxelGridReducesPoints)
{
    // Создаем тестовое облако - куб 10x10x10 точек
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 10; ++y)
            for (int z = 0; z < 10; ++z) {
                PointXYZRGB pt;
                pt.x = x * 0.01f;
                pt.y = y * 0.01f;
                pt.z = z * 0.01f;
                pt.r = 255; pt.g = 255; pt.b = 255;
                cloud->push_back(pt);
            }
    
    EXPECT_EQ(cloud->size(), 1000u);
    
    PointCloudFilters filters;
    CloudPtr filtered = filters.applyVoxelGrid(cloud, 0.05f);
    
    // При размере вокселя 0.05 должно получиться значительно меньше точек
    EXPECT_LT(filtered->size(), cloud->size());
    EXPECT_GT(filtered->size(), 0u);
}

// ========== Тест: Статистический фильтр ==========
TEST(FiltersTest, StatisticalFilterRemovesNoise)
{
    // Создаем облако с шумом
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Основные точки (сфера)
    for (int i = 0; i < 100; ++i) {
        PointXYZRGB pt;
        pt.x = cos(i * 0.1f) * 1.0f;
        pt.y = sin(i * 0.1f) * 1.0f;
        pt.z = 0.0f;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    // Добавляем выбросы (шум)
    for (int i = 0; i < 20; ++i) {
        PointXYZRGB pt;
        // Точки далеко от основных
        pt.x = (rand() % 1000) / 100.0f + 10.0f;
        pt.y = (rand() % 1000) / 100.0f + 10.0f;
        pt.z = (rand() % 1000) / 100.0f + 10.0f;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    PointCloudFilters filters;
    CloudPtr filtered = filters.applyStatisticalOutlierRemoval(cloud, 2, 0.5);
    
    // После фильтрации шум должен быть удален
    EXPECT_LT(filtered->size(), cloud->size());
    // Но основные точки должны остаться (около 100)
    EXPECT_GE(filtered->size(), 90u);
}

// ========== Тест: Граничный случай - пустое облако ==========
TEST(FiltersTest, EmptyCloudReturnsEmpty)
{
    CloudPtr empty(new pcl::PointCloud<PointXYZRGB>);
    EXPECT_TRUE(empty->empty());
    
    PointCloudFilters filters;
    
    // Воксельный фильтр на пустом облаке
    CloudPtr result = filters.applyVoxelGrid(empty, 0.01f);
    EXPECT_TRUE(result->empty());
    
    // Статистический фильтр
    result = filters.applyStatisticalOutlierRemoval(empty, 2, 0.5);
    EXPECT_TRUE(result->empty());
}

// ========== Тест: Граничный случай - одна точка ==========
TEST(FiltersTest, SinglePointCloud)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    PointXYZRGB pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    pt.r = 255; pt.g = 255; pt.b = 255;
    cloud->push_back(pt);
    
    EXPECT_EQ(cloud->size(), 1u);
    
    PointCloudFilters filters;
    CloudPtr filtered = filters.applyVoxelGrid(cloud, 0.01f);
    
    // Одна точка должна остаться
    EXPECT_EQ(filtered->size(), 1u);
}

// ========== Тест: Отрицательный размер вокселя ==========
TEST(FiltersTest, NegativeVoxelSizeReturnsInput)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    for (int i = 0; i < 10; ++i) {
        PointXYZRGB pt;
        pt.x = i * 0.1f; pt.y = 0; pt.z = 0;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    PointCloudFilters filters;
    
    // При отрицательном размере вокселя - возвращаем исходное облако
    CloudPtr filtered = filters.applyVoxelGrid(cloud, -1.0f);
    EXPECT_EQ(filtered->size(), cloud->size());
}

// ========== Тест: Обработка NaN значений ==========
TEST(FiltersTest, NaNPointsHandled)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Нормальные точки
    for (int i = 0; i < 50; ++i) {
        PointXYZRGB pt;
        pt.x = i * 0.1f;
        pt.y = i * 0.1f;
        pt.z = i * 0.1f;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    // Точки с NaN
    for (int i = 0; i < 10; ++i) {
        PointXYZRGB pt;
        pt.x = std::nan("");
        pt.y = std::nan("");
        pt.z = std::nan("");
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    PointCloudFilters filters;
    CloudPtr filtered = filters.applyVoxelGrid(cloud, 0.1f);
    
    // NaN точки должны быть отфильтрованы или не должны вызывать краш
    EXPECT_TRUE(filtered->empty() || !std::isnan(filtered->front().x));
}

// ========== Тест: PassThrough фильтр ==========
TEST(FiltersTest, PassThroughFilter)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Создаем точки в диапазоне [0, 2] по Z
    for (float z = 0.0f; z <= 2.0f; z += 0.1f) {
        PointXYZRGB pt;
        pt.x = 0; pt.y = 0; pt.z = z;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    PointCloudFilters filters;
    CloudPtr filtered = filters.applyPassThrough(cloud, "z", 0.5f, 1.5f);
    
    // Должны остаться точки с z в [0.5, 1.5]
    EXPECT_GE(filtered->size(), 10u);
    EXPECT_LE(filtered->size(), cloud->size());
    
    for (const auto& pt : filtered->points) {
        EXPECT_GE(pt.z, 0.5f);
        EXPECT_LE(pt.z, 1.5f);
    }
}
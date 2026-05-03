#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cmath>

using pcl::PointXYZRGB;
using CloudPtr = pcl::PointCloud<PointXYZRGB>::Ptr;

// Тест: Простое создание облака
TEST(SimpleCloudTest, CreatePointCloud)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    EXPECT_TRUE(cloud->empty());
    
    PointXYZRGB pt;
    pt.x = 1.0f; pt.y = 2.0f; pt.z = 3.0f;
    pt.r = 255; pt.g = 0; pt.b = 0;
    cloud->push_back(pt);
    
    EXPECT_EQ(cloud->size(), 1u);
}

// Тест: Воксельный фильтр PCL
TEST(SimpleFiltersTest, VoxelGridPCL)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Создаем 1000 точек (10x10x10)
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j)
            for (int k = 0; k < 10; ++k) {
                PointXYZRGB pt;
                pt.x = i * 0.01f;
                pt.y = j * 0.01f;
                pt.z = k * 0.01f;
                pt.r = 255; pt.g = 255; pt.b = 255;
                cloud->push_back(pt);
            }
    
    EXPECT_EQ(cloud->size(), 1000u);
    
    // VoxelGrid
    pcl::VoxelGrid<PointXYZRGB> vg;
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.setInputCloud(cloud);
    
    CloudPtr filtered(new pcl::PointCloud<PointXYZRGB>);
    vg.filter(*filtered);
    
    // После вокселизации меньше точек
    EXPECT_LT(filtered->size(), cloud->size());
    EXPECT_GT(filtered->size(), 0u);
}

// Тест: Стат. фильтр шума
TEST(SimpleFiltersTest, StatisticalOutlierRemoval)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    
    // Основные точки
    for (int i = 0; i < 100; ++i) {
        PointXYZRGB pt;
        pt.x = cos(i * 0.1f);
        pt.y = sin(i * 0.1f);
        pt.z = 0;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    // Добавляем шум (20 точек далеко)
    for (int i = 0; i < 20; ++i) {
        PointXYZRGB pt;
        pt.x = 100 + i;
        pt.y = 100 + i;
        pt.z = 100 + i;
        pt.r = 255; pt.g = 255; pt.b = 255;
        cloud->push_back(pt);
    }
    
    EXPECT_EQ(cloud->size(), 120u);
    
    // StatisticalOutlierRemoval
    pcl::StatisticalOutlierRemoval<PointXYZRGB> sor;
    sor.setMeanK(10);
    sor.setStdDevMulThresh(0.5);
    sor.setInputCloud(cloud);
    
    CloudPtr filtered(new pcl::PointCloud<PointXYZRGB>);
    sor.filter(*filtered);
    
    // Шум удален
    EXPECT_LT(filtered->size(), cloud->size());
}

// Тест: Пустое облако
TEST(SimpleFiltersTest, EmptyCloud)
{
    CloudPtr empty(new pcl::PointCloud<PointXYZRGB>);
    EXPECT_TRUE(empty->empty());
    
    pcl::VoxelGrid<PointXYZRGB> vg;
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.setInputCloud(empty);
    
    CloudPtr result(new pcl::PointCloud<PointXYZRGB>);
    vg.filter(*result);
    
    // Не краш - ожидаем пустой результат
    EXPECT_TRUE(result->empty());
}

// Тест: Одна точка
TEST(SimpleFiltersTest, SinglePoint)
{
    CloudPtr cloud(new pcl::PointCloud<PointXYZRGB>);
    PointXYZRGB pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    pt.r = 255; pt.g = 255; pt.b = 255;
    cloud->push_back(pt);
    
    EXPECT_EQ(cloud->size(), 1u);
    
    pcl::VoxelGrid<PointXYZRGB> vg;
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.setInputCloud(cloud);
    
    CloudPtr filtered(new pcl::PointCloud<PointXYZRGB>);
    vg.filter(*filtered);
    
    EXPECT_EQ(filtered->size(), 1u);
}
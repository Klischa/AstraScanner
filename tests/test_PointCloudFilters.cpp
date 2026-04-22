#include <gtest/gtest.h>
#include "filters/PointCloudFilters.h"
#include <cmath>

namespace {

// Создаёт куб из точек: N×N×N, шаг step метров.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeCube(int n, float step = 0.01f)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (int x = 0; x < n; ++x)
        for (int y = 0; y < n; ++y)
            for (int z = 0; z < n; ++z) {
                pcl::PointXYZRGB pt;
                pt.x = x * step;
                pt.y = y * step;
                pt.z = z * step;
                pt.r = 128; pt.g = 128; pt.b = 128;
                cloud->push_back(pt);
            }
    return cloud;
}

// Добавляет выбросы далеко от основного облака.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr addOutliers(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int count)
{
    auto result = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*cloud);
    for (int i = 0; i < count; ++i) {
        pcl::PointXYZRGB pt;
        pt.x = 10.0f + i;  // далеко от куба
        pt.y = 10.0f;
        pt.z = 10.0f;
        pt.r = 255; pt.g = 0; pt.b = 0;
        result->push_back(pt);
    }
    return result;
}

} // namespace

TEST(PointCloudFilters, SOR_RemovesOutliers)
{
    PointCloudFilters filters;
    auto cloud = addOutliers(makeCube(10), 5);
    const size_t before = cloud->size();

    auto filtered = filters.applyStatisticalOutlierRemoval(cloud, 20, 1.0);
    ASSERT_NE(filtered, nullptr);
    EXPECT_LT(filtered->size(), before);
    EXPECT_GT(filtered->size(), 0u);
}

TEST(PointCloudFilters, ROR_RemovesOutliers)
{
    PointCloudFilters filters;
    auto cloud = addOutliers(makeCube(10), 5);
    const size_t before = cloud->size();

    auto filtered = filters.applyRadiusOutlierRemoval(cloud, 0.05, 3);
    ASSERT_NE(filtered, nullptr);
    EXPECT_LT(filtered->size(), before);
}

TEST(PointCloudFilters, VoxelGrid_Decimates)
{
    PointCloudFilters filters;
    auto cloud = makeCube(20, 0.001f);  // 8000 точек, очень плотно
    const size_t before = cloud->size();

    auto filtered = filters.applyVoxelGrid(cloud, 0.005f);
    ASSERT_NE(filtered, nullptr);
    EXPECT_LT(filtered->size(), before);
    EXPECT_GT(filtered->size(), 0u);
}

TEST(PointCloudFilters, MagicWand_Combined)
{
    PointCloudFilters filters;
    auto cloud = addOutliers(makeCube(10), 10);
    const size_t before = cloud->size();

    auto filtered = filters.applyMagicWand(cloud);
    ASSERT_NE(filtered, nullptr);
    EXPECT_LT(filtered->size(), before);
}

TEST(PointCloudFilters, EmptyCloud_HandledGracefully)
{
    PointCloudFilters filters;
    auto empty = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    auto r1 = filters.applyStatisticalOutlierRemoval(empty);
    EXPECT_TRUE(!r1 || r1->empty());

    auto r2 = filters.applyVoxelGrid(empty);
    EXPECT_TRUE(!r2 || r2->empty());

    auto r3 = filters.applyRadiusOutlierRemoval(empty);
    EXPECT_TRUE(!r3 || r3->empty());
}

TEST(PointCloudFilters, NullCloud_HandledGracefully)
{
    PointCloudFilters filters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr null;

    auto r1 = filters.applyStatisticalOutlierRemoval(null);
    EXPECT_TRUE(!r1 || r1->empty());
}

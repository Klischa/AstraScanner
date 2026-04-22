#include <gtest/gtest.h>
#include "history/CloudHistory.h"

namespace {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeCloud(int n)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (int i = 0; i < n; ++i) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(i);
        pt.y = 0; pt.z = 0;
        pt.r = 255; pt.g = 0; pt.b = 0;
        cloud->push_back(pt);
    }
    return cloud;
}

} // namespace

TEST(CloudHistory, PushAndUndo)
{
    CloudHistory h;
    auto c1 = makeCloud(10);
    auto c2 = makeCloud(20);

    h.pushState(c1, "op1");
    EXPECT_TRUE(h.canUndo());
    EXPECT_FALSE(h.canRedo());
    EXPECT_EQ(h.undoDepth(), 1);

    // Undo должен вернуть c1 (10 точек)
    auto restored = h.undo(c2);
    ASSERT_NE(restored, nullptr);
    EXPECT_EQ(restored->size(), 10u);
    EXPECT_FALSE(h.canUndo());
    EXPECT_TRUE(h.canRedo());
}

TEST(CloudHistory, Redo)
{
    CloudHistory h;
    auto c1 = makeCloud(10);
    auto c2 = makeCloud(20);
    auto c3 = makeCloud(30);

    h.pushState(c1, "op1");
    h.pushState(c2, "op2");

    auto afterUndo = h.undo(c3);
    ASSERT_NE(afterUndo, nullptr);
    EXPECT_EQ(afterUndo->size(), 20u);

    auto afterRedo = h.redo(afterUndo);
    ASSERT_NE(afterRedo, nullptr);
    EXPECT_EQ(afterRedo->size(), 30u);
}

TEST(CloudHistory, PushClearsRedo)
{
    CloudHistory h;
    auto c1 = makeCloud(10);
    auto c2 = makeCloud(20);

    h.pushState(c1, "op1");
    h.undo(c2);
    EXPECT_TRUE(h.canRedo());

    h.pushState(makeCloud(5), "op2");
    EXPECT_FALSE(h.canRedo());
}

TEST(CloudHistory, MaxDepth)
{
    CloudHistory h;
    h.setMaxDepth(3);

    for (int i = 0; i < 10; ++i) {
        h.pushState(makeCloud(i + 1), QString("op%1").arg(i));
    }
    EXPECT_EQ(h.undoDepth(), 3);
}

TEST(CloudHistory, Clear)
{
    CloudHistory h;
    h.pushState(makeCloud(10), "op1");
    h.pushState(makeCloud(20), "op2");
    h.clear();
    EXPECT_FALSE(h.canUndo());
    EXPECT_FALSE(h.canRedo());
    EXPECT_EQ(h.undoDepth(), 0);
}

TEST(CloudHistory, EmptyCloudIgnored)
{
    CloudHistory h;
    auto empty = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    h.pushState(empty, "empty");
    EXPECT_FALSE(h.canUndo());

    h.pushState(nullptr, "null");
    EXPECT_FALSE(h.canUndo());
}

TEST(CloudHistory, UndoOnEmptyReturnsNull)
{
    CloudHistory h;
    auto result = h.undo(makeCloud(10));
    EXPECT_EQ(result, nullptr);
}

#include <gtest/gtest.h>

#include "visual_inertial_frontend/tracks_buffer.hpp"

#include <opencv2/core.hpp>

#include <vector>

TEST(TracksBufferTest, GatherPnPAndGateByPnPInliersKeepPendingTracks)
{
    TracksBuffer buffer;
    buffer.addNewLeft({{1.0f, 1.0f}, {2.0f, 2.0f}, {3.0f, 3.0f}});
    const std::vector<uint8_t> valid{1, 1, 0};
    buffer.setPrev3DAll(
        {{1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}},
        &valid);

    std::vector<cv::Point3f> object_pts;
    std::vector<cv::Point2f> image_pts;
    std::vector<int> indices;
    buffer.gatherPnP(object_pts, image_pts, &indices);

    ASSERT_EQ(2u, object_pts.size());
    EXPECT_EQ((std::vector<int>{0, 1}), indices);

    buffer.gateByPnPInliers(indices, {1});

    ASSERT_EQ(2u, buffer.size());
    EXPECT_EQ((std::vector<cv::Point2f>{{2.0f, 2.0f}, {3.0f, 3.0f}}), buffer.pl());
    EXPECT_EQ(std::vector<uint8_t>({1, 0}), buffer.hasXprev());
}

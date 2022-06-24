#pragma once

#include "Base64.h"
#include "Image.h"
#include "Topics.h"

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"

#include "hololens_msgs/PixelDirection.h"
#include "hololens_msgs/Point.h"
#include "hololens_msgs/Quaternion.h"
#include "hololens_msgs/StereoCameraFrame.h"
#include "hololens_msgs/StereoPixelDirections.h"

#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <hololens_stereo_image_receiver/StereoImageReceiverConfig.h>

class StereoImageReceiver
{
public:
    StereoImageReceiver(ros::NodeHandle n);

    // Callback for handling the incoming stereo camera frames and dynamic reconfiguration.
    void handleStereoCameraFrame(const hololens_msgs::StereoCameraFrame::ConstPtr& msg);
    void handleReconfiguration(hololens_stereo_image_receiver::StereoImageReceiverConfig& config, uint32_t level);

private:
    // Methods for publishing the results.
    sensor_msgs::Image imageToMsg(
        const Image image,
        const std::string frameId,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
    cv_bridge::CvImage imageToMsg(
        const cv::Mat& image,
        const std::string frameId,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
    void publishImage(
        const Image image,
        const ros::Publisher& publisher,
        const std::string frameId,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
    void publishDisparityMap(
        const cv::Mat& disparityMap,
        const ros::Publisher& publisher,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
    void publishHololensPosition(
        const hololens_msgs::Point& position,
        const ros::Publisher& publisher,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
    void publishHololensCamToWorldTf(
        const hololens_msgs::Point& translation,
        const hololens_msgs::Quaternion& rotation,
        const std::string frameId,
        tf::TransformBroadcaster& publisher,
        const ros::Time& timestamp);

    // Sensor intrinsics of the stereo cameras.
    cv::Mat RLeft, RRight, PLeft, PRight, Q, KLeft, KRight, R, DLeft, DRight;
    cv::Vec3d T;
    cv::Mat map1Left, map1Right, map2Left, map2Right;
    bool undistortRectifyMapInitialized;
    cv::Rect validPixelsRectLeft;
    cv::Rect validPixelsRectRight;

    // Parameters for preprocessing the raw stereo images.
    bool preprocessingDoImageNormalization;
    bool preprocessingDoHistogramEqualization;

    // Parameters for OpenCV's StereoSGBM algorithm.
    int sgbmMinDisparity;
    int sgbmNumDisparities;
    int sgbmBlockSize;
    int sgbmP1Multiplier;
    int sgbmP2Multiplier;
    int sgbmDisp12MaxDiff;
    int sgbmPreFilterCap;
    int sgbmUniquenessRatio;
    int sgbmSpeckleWindowSize;
    int sgbmSpeckleRange;
    bool sgbmUseModeHH;
    bool updateMatchers = false;
    cv::Ptr<cv::StereoSGBM> leftMatcher;
    cv::Ptr<cv::StereoMatcher> rightMatcher;

    // Parameters for OpenCV's DisparityWLSFilter.
    double wlsLambda;
    double wlsSigma;
    bool updateWlsFilter = false;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter;

    // Parameters for the disparity map visualization.
    double dispVisMultiplier;

    // Parameters for reconstruction of a point cloud from the disparity map.
    bool reconstructPointCloudFromRawDisparityMap;
    float minDisparityForReconstruction;

    // ROS publishers.
    ros::Publisher stereoImageLeftRawPublisher;
    ros::Publisher stereoImageRightRawPublisher;
    ros::Publisher stereoImageLeftPublisher;
    ros::Publisher stereoImageRightPublisher;
    ros::Publisher disparityMapRawPublisher;
    ros::Publisher disparityMapPublisher;
    ros::Publisher stereoCamLeftPositionPublisher;
    ros::Publisher stereoCamRightPositionPublisher;
    ros::Publisher hololensPositionPublisher;
    ros::Publisher pointCloudPublisher;
    tf::TransformBroadcaster hololensCamLeftPublisher;
    tf::TransformBroadcaster hololensCamRightPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t sequenceNumber;
};

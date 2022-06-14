#pragma once

#include "Base64.h"
#include "Image.h"
#include "Topics.h"

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"

#include "hololens_msgs/PixelDirection.h"
#include "hololens_msgs/Point.h"
#include "hololens_msgs/Quaternion.h"
#include "hololens_msgs/StereoCameraFrame.h"
#include "hololens_msgs/StereoPixelDirections.h"

#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"

class StereoImageReceiver
{
public:
    StereoImageReceiver(ros::NodeHandle n);

    // Callbacks for handling the incoming stereo camera frames and pixel directions.
    void handleStereoCameraFrame(const hololens_msgs::StereoCameraFrame::ConstPtr& msg);
    void handleStereoPixelDirections(const hololens_msgs::StereoPixelDirections::ConstPtr& msg);

private:
    // Methods for publishing the results.
    sensor_msgs::Image imageToMsg(
        const Image image,
        const std::string frameId,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
    void publishImage(
        const Image image,
        const ros::Publisher& publisher,
        const std::string frameId,
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
    cv::Ptr<cv::StereoSGBM> leftMatcher;
    cv::Ptr<cv::StereoMatcher> rightMatcher;

    // Parameters for OpenCV's DisparityWLSFilter.
    double wlsLambda;
    double wlsSigma;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter;

    // Parameters for the disparity map visualization.
    double dispVisMultiplier;
    bool dispVisVisualizeRawDisparityMap;

    // The directions (in camera space) in which each pixel of the stereo frames points at.
    hololens_msgs::StereoPixelDirections::Ptr stereoPixelDirections;

    // ROS publishers.
    ros::Publisher stereoImageLeftPublisher;
    ros::Publisher stereoImageRightPublisher;
    ros::Publisher disparityMapPublisher;
    ros::Publisher stereoCamLeftPositionPublisher;
    ros::Publisher stereoCamRightPositionPublisher;
    ros::Publisher hololensPositionPublisher;
    tf::TransformBroadcaster hololensCamLeftPublisher;
    tf::TransformBroadcaster hololensCamRightPublisher;

    // Sequence numbers used for publishing the results.
    uint32_t sequenceNumber;
};

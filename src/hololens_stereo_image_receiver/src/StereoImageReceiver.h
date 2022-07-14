#pragma once

#include "Base64.h"
#include "Image.h"
#include "Topics.h"

#include "ros/ros.h"

#include "camera_calibration_parsers/parse.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/CameraInfo.h"
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
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
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
    // Downsamples a given point cloud.
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToDownsample,
        const float leafSize);
    
    // Removes outliers from a given point cloud with a radius outlier removal filter.
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeOutliersRadius(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToFilter,
        const float radiusSearch,
        const int minNeighborsInRadius);

    // Removes outliers from a given point cloud with a statistical outlier removal filter.
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeOutliersStatistical(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToFilter,
        const float numNeighborsToCheck,
        const float standardDeviationMultiplier);

    // Removes outliers from a given point cloud by clustering the points and discarding too small clusters.
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeOutliersClustering(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudToFilter,
        const double clusterTolerance,
        const int minClusterSize);

    // Computes the transformation matrix for transforming from camera space to world space.
    Eigen::Matrix4f computeCamToWorldFromStereoCameraFrame(
        const hololens_msgs::StereoCameraFrame::ConstPtr& stereoCamFrame);

    // Methods for publishing the results.
    void publishCameraInfo(
        sensor_msgs::CameraInfo& cameraInfo,
        const ros::Publisher& publisher,
        const std::string frameId,
        uint32_t sequenceNumber,
        const ros::Time& timestamp);
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
    void pointCloudToMsg(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        sensor_msgs::PointCloud2& message,
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const std::string frameId);
    void publishPointCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const ros::Publisher& publisher,
        uint32_t sequenceNumber,
        const ros::Time& timestamp,
        const std::string frameId);
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

    std::string leftCameraName;
    std::string rightCameraName;
    sensor_msgs::CameraInfo leftCameraInfo;
    sensor_msgs::CameraInfo rightCameraInfo;

    // Parameters for preprocessing the raw stereo images.
    bool preprocessingDoImageNormalization;
    bool preprocessingDoHistogramEqualization;
    bool preprocessingDoMedianFiltering;
    int preprocessingMedianFilterKernelSize;

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
    int sgbmMode;
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

    // Parameters for point cloud downsampling.
    bool doDownsampling;
    float downsamplingLeafSize;

    // Parameters for point cloud outlier removal using a radius outlier removal filter.
    bool doOutlierRemovalRadius;
    double outlierRemovalRadiusSearch;
    int outlierRemovalMinNeighborsInRadius;

    // Parameters for point cloud outlier removal using a statistical outlier removal filter.
    bool doOutlierRemovalStatistical;
    int outlierRemovalNeighborsToCheck;
    double outlierRemovalStdDeviationMultiplier;

    // Parameters for point cloud outlier removal using euclidean clustering.
    bool doOutlierRemovalClustering;
    double outlierRemovalClusterTolerance;
    int outlierRemovalMinClusterSize;

    // ROS publishers.
    ros::Publisher cameraInfoLeftPublisher;
    ros::Publisher cameraInfoRightPublisher;
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

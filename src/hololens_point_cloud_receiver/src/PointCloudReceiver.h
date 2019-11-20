#include <string>

#include "Base64.h"

#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"

#include "hololens_point_cloud_msgs/DepthFrame.h"
#include "hololens_point_cloud_msgs/Matrix.h"
#include "hololens_point_cloud_msgs/PixelDirection.h"
#include "hololens_point_cloud_msgs/PixelDirections.h"
#include "hololens_point_cloud_msgs/Point.h"
#include "hololens_point_cloud_msgs/PointCloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define SHORT_THROW_MIN_RELIABLE_DEPTH 0.2f
#define SHORT_THROW_MAX_RELIABLE_DEPTH 1.0f

#define LONG_THROW_MIN_RELIABLE_DEPTH 0.5f
#define LONG_THROW_MAX_RELIABLE_DEPTH 4.0f

#define DOWNSAMPLING_LEAF_SIZE 0.01f

#define SHORT_THROW_PIXEL_DIRECTIONS_TOPIC "/hololensShortThrowPixelDirections"
#define LONG_THROW_PIXEL_DIRECTIONS_TOPIC "/hololensLongThrowPixelDirections"

#define SHORT_THROW_DEPTH_TOPIC "/hololensShortThrowDepth"
#define LONG_THROW_DEPTH_TOPIC "/hololensLongThrowDepth"

#define SHORT_THROW_IMAGE_TOPIC "/hololensShortThrowImage"
#define LONG_THROW_IMAGE_TOPIC "/hololensLongThrowImage"

#define CLEAR_POINT_CLOUD_TOPIC "/clearPointCloud"
#define SAVE_POINT_CLOUD_TOPIC "/savePointCloud"

#define POINT_CLOUD_TOPIC "/pointCloud"

class PointCloudReceiver
{
public:
    PointCloudReceiver(ros::NodeHandle n);

    void handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);
    void handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);

    void clearPointCloud();
    void savePointCloud();

private:
    void handleDepthFrame(
        const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame, 
        const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
        const float minReliableDepth,
        const float maxReliableDepth,
        const ros::Publisher& imagePublisher,
        uint32_t* sequenceNumber);

private:
    hololens_point_cloud_msgs::PixelDirections::ConstPtr shortThrowDirections;
    hololens_point_cloud_msgs::PixelDirections::ConstPtr longThrowDirections;

    ros::Publisher shortThrowImagePublisher;
    ros::Publisher longThrowImagePublisher;
    ros::Publisher pointCloudPublisher;

    uint32_t shortThrowSequenceNumber;
    uint32_t longThrowSequenceNumber;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;

    boost::mutex pointCloudMutex;
};
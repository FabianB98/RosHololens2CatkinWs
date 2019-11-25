#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include "hololens_point_cloud_msgs/DepthFrame.h"
#include "hololens_point_cloud_msgs/Matrix.h"
#include "hololens_point_cloud_msgs/PixelDirection.h"
#include "hololens_point_cloud_msgs/PixelDirections.h"
#include "hololens_point_cloud_msgs/Point.h"
#include "hololens_point_cloud_msgs/PointCloud.h"

class DepthFrameRecorder
{
public:
    DepthFrameRecorder();

    void handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg);
    void handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);
    void handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg);

    void handleRecord(const std_msgs::Bool::ConstPtr& msg);

private:
    void handleDepthFrame(
        const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame,
        const bool isLongThrow);

    void startRecording();
    void finishRecording();

    void savePixelDirections(
        const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
        const std::string pixelDirectionsFilePath);

    void saveDepthFrame(
        const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame,
        const bool isLongThrow,
        const std::string frameNumber);

private:
    hololens_point_cloud_msgs::PixelDirections::ConstPtr shortThrowDirections;
    hololens_point_cloud_msgs::PixelDirections::ConstPtr longThrowDirections;

    bool recording;
    std::string depthFrameDirectory;

    boost::mutex frameMutex;
};
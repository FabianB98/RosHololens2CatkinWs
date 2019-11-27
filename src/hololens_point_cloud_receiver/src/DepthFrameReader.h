#include <iostream>
#include <string>
#include <sstream>

#include "SpatialMapper.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "hololens_point_cloud_msgs/DepthFrame.h"
#include "hololens_point_cloud_msgs/PixelDirections.h"

class DepthFrameReader
{
public:
    DepthFrameReader(SpatialMapper* mapper);

    void processRecording(const std_msgs::String::ConstPtr& msg);

private:
    hololens_point_cloud_msgs::PixelDirections::ConstPtr loadPixelDirections(std::string filename);
    hololens_point_cloud_msgs::DepthFrame::ConstPtr loadDepthFrame(const boost::filesystem::path& path, bool* isLongThrow);

private:
    SpatialMapper* spatialMapper;
};
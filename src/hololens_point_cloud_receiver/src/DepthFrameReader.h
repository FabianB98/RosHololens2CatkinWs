#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>

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
    DepthFrameReader(SpatialMapper* mappers[], int size);

    void processRecording(const std_msgs::String::ConstPtr& msg);

private:
    hololens_point_cloud_msgs::PixelDirections::ConstPtr loadPixelDirections(std::string filename);
    hololens_point_cloud_msgs::DepthFrame::ConstPtr loadDepthFrame(const boost::filesystem::path& path, bool* isLongThrow);

private:
    SpatialMapper** spatialMappers;
    int numMappers;
};
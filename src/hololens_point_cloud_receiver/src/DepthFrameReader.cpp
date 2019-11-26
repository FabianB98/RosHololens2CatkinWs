#include "DepthFrameReader.h"

DepthFrameReader::DepthFrameReader(SpatialMapper* mapper)
{
    spatialMapper = mapper;
}

void DepthFrameReader::processRecording(const std_msgs::String::ConstPtr& msg)
{
    // Get all relevant paths.
    std::string recordingDirectory = msg->data;
    std::string depthFrameDirectory = recordingDirectory + "/depthFrames/";
    std::string shortThrowPixelDirsFile = recordingDirectory + "/pixelDirections/shortThrow.pixeldirs";
    std::string longThrowPixelDirsFile = recordingDirectory + "/pixelDirections/longThrow.pixeldirs";
    ROS_INFO("Processing recording \"%s\"...", recordingDirectory.c_str());

    // Load the pixel directions and process them with the spatial mapper.
    spatialMapper->handleShortThrowPixelDirections(loadPixelDirections(shortThrowPixelDirsFile));
    spatialMapper->handleLongThrowPixelDirections(loadPixelDirections(longThrowPixelDirsFile));

    // TODO: Iterate over each depth frame, load that depth frame and process it.

    ROS_INFO("Finished processing recording \"%s\"!", recordingDirectory.c_str());
}

hololens_point_cloud_msgs::PixelDirections::ConstPtr DepthFrameReader::loadPixelDirections(std::string filename)
{
    hololens_point_cloud_msgs::PixelDirections::ConstPtr pixelDirections(new hololens_point_cloud_msgs::PixelDirections());

    boost::filesystem::ifstream file(filename);
    std::string line;

    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string substring;

        std::getline(ss, substring, ';');
        ROS_INFO("U: %s", substring.c_str());
        std::getline(ss, substring, ';');
        ROS_INFO("V: %s", substring.c_str());
        std::getline(ss, substring, ';');
        ROS_INFO("X: %s", substring.c_str());
        std::getline(ss, substring, ';');
        ROS_INFO("Y: %s", substring.c_str());
        std::getline(ss, substring, ';');
        ROS_INFO("Z: %s", substring.c_str());

        // TODO: Finish implementing this method...
    }

    file.close();

    return pixelDirections;
}

hololens_point_cloud_msgs::DepthFrame::ConstPtr DepthFrameReader::loadDepthFrame(std::string filename)
{
    hololens_point_cloud_msgs::DepthFrame::ConstPtr depthFrame(new hololens_point_cloud_msgs::DepthFrame());

    // TODO: Implement this method...

    return depthFrame;
}
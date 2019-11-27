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

    // Get information about all stored depth frames and sort them in in lexical order. Sorting the frames in
    // lexical order is done to assert that the frames will be processed in the order in which they were captured.
    std::vector<boost::filesystem::path> v;
    boost::filesystem::directory_iterator dirItr(depthFrameDirectory);
    boost::filesystem::directory_iterator endItr;
    std::copy(dirItr, endItr, std::back_inserter(v));
    std::sort(v.begin(), v.end());

    // Iterate over each stored depth frame.
    for (std::vector<boost::filesystem::path>::const_iterator itr(v.begin()), itr_end(v.end()); itr != itr_end; ++itr)
    {
        // Load the current depth frame.
        bool isLongThrow;
        hololens_point_cloud_msgs::DepthFrame::ConstPtr depthFrame = loadDepthFrame(*itr, &isLongThrow);

        // Process the current depth frame.
        if (isLongThrow)
            spatialMapper->handleLongThrowDepthFrame(depthFrame);
        else
            spatialMapper->handleShortThrowDepthFrame(depthFrame);
    }

    ROS_INFO("Finished processing recording \"%s\"!", recordingDirectory.c_str());
}

hololens_point_cloud_msgs::PixelDirections::ConstPtr DepthFrameReader::loadPixelDirections(std::string filename)
{
    // Create the pixel directions instance which will be returned later on.
    hololens_point_cloud_msgs::PixelDirections* pixelDirections = new hololens_point_cloud_msgs::PixelDirections();

    // Open the file in which the pixel directions are stored and iterate over each line of the file.
    boost::filesystem::ifstream file(filename);
    std::string line;
    while(std::getline(file, line))
    {
        // Create a string stream for splitting the current line into the components of the pixel direction.
        std::stringstream ss(line);
        std::string u, v, x, y, z;

        // Split the current line into the components (u, v, x, y and z) of the pixel direction.
        std::getline(ss, u, ';');
        std::getline(ss, v, ';');
        std::getline(ss, x, ';');
        std::getline(ss, y, ';');
        std::getline(ss, z, ';');

        // Create the pixel direction instance for the current pixel direction.
        hololens_point_cloud_msgs::PixelDirection pixelDirection;
        pixelDirection.u = boost::lexical_cast<uint32_t>(u);
        pixelDirection.v = boost::lexical_cast<uint32_t>(v);
        pixelDirection.direction.x = boost::lexical_cast<float>(x);
        pixelDirection.direction.y = boost::lexical_cast<float>(y);
        pixelDirection.direction.z = boost::lexical_cast<float>(z);

        // Add the pixel direction instance to the loaded pixel directions.
        pixelDirections->pixelDirections.push_back(pixelDirection);
    }

    // Close the file and return the loaded pixel directions.
    file.close();
    return hololens_point_cloud_msgs::PixelDirections::ConstPtr(pixelDirections);
}

hololens_point_cloud_msgs::DepthFrame::ConstPtr DepthFrameReader::loadDepthFrame(
    const boost::filesystem::path& path, bool* isLongThrow)
{
    // Open the file in which the depth frame is stored for splitting it into the components of the depth frame.
    boost::filesystem::ifstream file(path);
    std::string longThrow, width, height, pixelStride, depthMap, tx, ty, tz, rx, ry, rz, rw;

    // Split the file into the components (width, height, pixel stride, depth map and transformation) of the frame.
    std::getline(file, longThrow);
    std::getline(file, width);
    std::getline(file, height);
    std::getline(file, pixelStride);
    std::getline(file, depthMap);
    std::getline(file, tx, ';');
    std::getline(file, ty, ';');
    std::getline(file, tz);
    std::getline(file, rx, ';');
    std::getline(file, ry, ';');
    std::getline(file, rz, ';');
    std::getline(file, rw);

    // Close the file.
    file.close();

    // Create the depth frame instance which will be returned later on.
    hololens_point_cloud_msgs::DepthFrame* depthFrame = new hololens_point_cloud_msgs::DepthFrame();

    // Set the depth map and all relevant meta data of the depth frame.
    depthFrame->depthMapWidth = boost::lexical_cast<uint32_t>(width);
    depthFrame->depthMapHeight = boost::lexical_cast<uint32_t>(height);
    depthFrame->depthMapPixelStride = boost::lexical_cast<uint32_t>(pixelStride);
    depthFrame->base64encodedDepthMap = depthMap;

    // Set the translation of the depth frame.
    depthFrame->camToWorldTranslation.x = boost::lexical_cast<float>(tx);
    depthFrame->camToWorldTranslation.y = boost::lexical_cast<float>(ty);
    depthFrame->camToWorldTranslation.z = boost::lexical_cast<float>(tz);

    // Set the rotation of the depth frame.
    depthFrame->camToWorldRotation.x = boost::lexical_cast<float>(rx);
    depthFrame->camToWorldRotation.y = boost::lexical_cast<float>(ry);
    depthFrame->camToWorldRotation.z = boost::lexical_cast<float>(rz);
    depthFrame->camToWorldRotation.w = boost::lexical_cast<float>(rw);
    
    // Set the isLongThrow result parameter and return the loaded depth frame.
    *isLongThrow = boost::lexical_cast<bool>(longThrow);
    return hololens_point_cloud_msgs::DepthFrame::ConstPtr(depthFrame);
}
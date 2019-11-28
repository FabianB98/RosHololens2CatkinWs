#include "DepthFrameRecorder.h"

DepthFrameRecorder::DepthFrameRecorder()
{
    ROS_INFO("Creating DepthFrameRecorder...");

    recording = false;
}

void DepthFrameRecorder::handleShortThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a short throw depth frame!");
    handleDepthFrame(msg, false);
}

void DepthFrameRecorder::handleLongThrowDepthFrame(const hololens_point_cloud_msgs::DepthFrame::ConstPtr& msg)
{
    ROS_INFO("Received a long throw depth frame!");
    handleDepthFrame(msg, true);
}

void DepthFrameRecorder::handleDepthFrame(
    const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame,
    const bool isLongThrow)
{
    // Get the current time.
    uint64_t currentTime = ros::Time::now().toNSec();

    // Check whether we should save this depth frame.
    frameMutex.lock();
    bool saveFrame = recording;
    frameMutex.unlock();

    // Save the depth frame if needed.
    if (saveFrame)
        saveDepthFrame(depthFrame, isLongThrow, boost::lexical_cast<std::string>(currentTime));
}

void DepthFrameRecorder::handleShortThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu short throw pixel directions!", msg->pixelDirections.size());
    shortThrowDirections = msg;
}

void DepthFrameRecorder::handleLongThrowPixelDirections(const hololens_point_cloud_msgs::PixelDirections::ConstPtr& msg)
{
    ROS_INFO("Received %zu long throw pixel directions!", msg->pixelDirections.size());
    longThrowDirections = msg;
}

void DepthFrameRecorder::handleRecord(const std_msgs::Bool::ConstPtr& msg)
{
    // Lock the frame mutex as we're about to enter a critical section.
    frameMutex.lock();

    // Check whether we should start recording.
    if (!recording && msg->data) 
        startRecording();

    // Check whether we should stop recording.
    if (recording && !msg->data)
        finishRecording();

    // Store whether we are recording and unlock the frame mutex again.
    recording = msg->data;
    frameMutex.unlock();
}

void DepthFrameRecorder::startRecording()
{
    ROS_INFO("Starting recording...");

    // Create the directory in which everything from this recording will be stored.
    std::string home = std::string(getenv("HOME"));
    std::string directory = home + "/recordings/" + boost::lexical_cast<std::string>(ros::Time::now().toNSec()) + "/";
    boost::filesystem::create_directories(directory);

    // Create the subdirectory for the pixel directions and store the short and long throw pixel directions.
    std::string pixelDirectionsDirectory = directory + "pixelDirections/";
    boost::filesystem::create_directories(pixelDirectionsDirectory);
    savePixelDirections(shortThrowDirections, pixelDirectionsDirectory + "shortThrow.pixeldirs");
    savePixelDirections(longThrowDirections, pixelDirectionsDirectory + "longThrow.pixeldirs");

    // Create the subdirectory for the depth frames.
    depthFrameDirectory = directory + "depthFrames/";
    boost::filesystem::create_directories(depthFrameDirectory);
}

void DepthFrameRecorder::finishRecording()
{
    ROS_INFO("Finishing recording...");
}

void DepthFrameRecorder::savePixelDirections(
    const hololens_point_cloud_msgs::PixelDirections::ConstPtr& pixelDirections,
    const std::string pixelDirectionsFilePath)
{
    // Open the file in which we will store the pixel directions.
    boost::filesystem::ofstream file(pixelDirectionsFilePath);

    // Write all pixel directions into the file.
    for (uint32_t i = 0; i < pixelDirections->pixelDirections.size(); ++i)
    {
        // Get the current pixel direction.
        hololens_point_cloud_msgs::PixelDirection dir = pixelDirections->pixelDirections.at(i);

        // Write the current pixel direction into the file.
        file << boost::lexical_cast<std::string>(dir.u);
        file << ";";
        file << boost::lexical_cast<std::string>(dir.v);
        file << ";";
        file << boost::lexical_cast<std::string>(dir.direction.x);
        file << ";";
        file << boost::lexical_cast<std::string>(dir.direction.y);
        file << ";";
        file << boost::lexical_cast<std::string>(dir.direction.z);
        file << "\n";
    }
    
    // Close the file.
    file.close();
}

void DepthFrameRecorder::saveDepthFrame(
    const hololens_point_cloud_msgs::DepthFrame::ConstPtr& depthFrame,
    const bool isLongThrow,
    const std::string frameNumber)
{
    // Open the file in which we will store the depth frame.
    boost::filesystem::ofstream file(depthFrameDirectory + frameNumber + ".frame");

    // Store whether this is a long throw depth frame.
    file << boost::lexical_cast<std::string>(isLongThrow);
    file << "\n";

    // Store the depth map and all of its relevant meta data.
    file << boost::lexical_cast<std::string>(depthFrame->depthMapWidth);
    file << "\n";
    file << boost::lexical_cast<std::string>(depthFrame->depthMapHeight);
    file << "\n";
    file << boost::lexical_cast<std::string>(depthFrame->depthMapPixelStride);
    file << "\n";
    file << depthFrame->base64encodedDepthMap;
    file << "\n";

    // Store the cam to world translation (i.e. the position of the HoloLens).
    file << boost::lexical_cast<std::string>(depthFrame->camToWorldTranslation.x);
    file << ";";
    file << boost::lexical_cast<std::string>(depthFrame->camToWorldTranslation.y);
    file << ";";
    file << boost::lexical_cast<std::string>(depthFrame->camToWorldTranslation.z);
    file << "\n";

    // Store the cam to world rotation (i.e. the direction in which the HoloLens is pointed at).
    file << boost::lexical_cast<std::string>(depthFrame->camToWorldRotation.x);
    file << ";";
    file << boost::lexical_cast<std::string>(depthFrame->camToWorldRotation.y);
    file << ";";
    file << boost::lexical_cast<std::string>(depthFrame->camToWorldRotation.z);
    file << ";";
    file << boost::lexical_cast<std::string>(depthFrame->camToWorldRotation.w);
    file << "\n";

    // Close the file.
    file.close();
}
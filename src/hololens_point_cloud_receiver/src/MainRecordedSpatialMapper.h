#include "DepthFrameReader.h"
#include "Topics.h"

DepthFrameReader* depthFrameReader;
SpatialMapper* spatialMapper;

int main(int argc, char **argv);

void loadRecordingCallback(const std_msgs::String::ConstPtr& msg);

void clearPointCloudCallback(const std_msgs::Bool::ConstPtr& msg);
void savePointCloudCallback(const std_msgs::Bool::ConstPtr& msg);
void findPlanesCallback(const std_msgs::Bool::ConstPtr& msg);

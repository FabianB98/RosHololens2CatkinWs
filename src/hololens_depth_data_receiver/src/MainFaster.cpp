#include "MainInitDepthDataReceiver.h"
#include "FasterDepthDataReceiver.h"

DepthDataReceiver* initializeDepthDataReceiver(ros::NodeHandle& n)
{
    return new FasterDepthDataReceiver(n);
}

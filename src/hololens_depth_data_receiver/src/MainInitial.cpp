#include "MainInitDepthDataReceiver.h"
#include "InitialDepthDataReceiver.h"

DepthDataReceiver* initializeDepthDataReceiver(ros::NodeHandle& n)
{
    return new InitialDepthDataReceiver(n);
}

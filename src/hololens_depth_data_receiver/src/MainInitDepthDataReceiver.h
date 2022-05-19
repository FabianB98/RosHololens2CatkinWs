#pragma once

#include "ros/ros.h"
#include "DepthDataReceiver.h"

// I don't quite understand why this method has to be in a separate header file, but when I define it in Main.h and
// include Main.h in MainInitial.cpp and MainFaster.cpp, it won't compile anymore due to depthDataReceiver being defined
// muliple times (even though it is only defined once in Main.h?!)
DepthDataReceiver* initializeDepthDataReceiver(ros::NodeHandle& n);

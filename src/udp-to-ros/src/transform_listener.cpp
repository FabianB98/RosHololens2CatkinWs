#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <iostream>

double toDeg(double rad) 
{ 
    double pi = 3.14159265359; 
    return (rad * (180 / pi)); 
} 

int main(int argc, char** argv){
  ros::init(argc, argv, "transform_listener");

  tf::TransformListener listener;
  tf::Transform totransform1, totransform2;
  totransform1.setIdentity();
  totransform2.setIdentity();
  
  tf::Vector3 totPos1 = tf::Vector3(0, 0, 0);
  tf::Vector3 totPos2 = tf::Vector3(0, 0, 0);
  tf::Quaternion totRot1(0,0,0,0);
  tf::Quaternion totRot2(0,0,0,0);

  ros::Rate rate(10.0);
  long long count = 0;
  while (1){
    tf::StampedTransform transform1, transform2;
    
    try{
      listener.lookupTransform("/ART_pen", "/markers", ros::Time(0), transform1);
      //listener.lookupTransform("/world", "/recalc_world", ros::Time(0), transform2);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    count++;
    
    tf::Vector3 pos1  = transform1.getOrigin();
    tf::Quaternion rot1 = transform1.getRotation();
    
    //tf::Vector3 pos2  = transform2.getOrigin();
    //tf::Quaternion rot2 = transform2.getRotation();
    
    float x,y,z,rx,ry,rz,rw;
    x = pos1.x();
    y = pos1.y();
    z = pos1.z();
    rx = rot1.x();
    ry = rot1.y();
    rz = rot1.z();
    rw = rot1.w();
    
    totPos1 += tf::Vector3(std::abs(x), std::abs(y), std::abs(z));
    //totPos2 += pos2;
    
    totRot1 += tf::Quaternion(std::abs(rx), std::abs(ry), std::abs(rz), std::abs(rw));
    //totRot2 += rot2;
    
    pos1 = totPos1/count;
    //pos2 = totPos2/count;
    rot1 = totRot1/count;
    //rot2 = totRot2/count;
    
    tf::Matrix3x3 erMat(rot1);
    tfScalar yaw, pitch, roll;

    erMat.getEulerYPR(yaw, pitch, roll);

    std::cout << "\nSample Count: " << count << "\n";
    //printf("\nTimestamp: ");
    //std::cout << ros::Time::now() << "\n";
    
    std::cout << "\nART_pen to markers absolute mean transformation\n";
    printf("\nX: %f \nY: %f\nZ: %f\n", pos1.x()*1000, pos1.y()*1000, pos1.z()*1000);
    printf("\nrX[°]: %f \nrY[°]: %f\nrZ[°]: %f \n", toDeg(roll), toDeg(pitch), toDeg(yaw));
    
    std::cout << "\nMean position error(mm): \n";
    std::cout << (pos1.x() + pos1.y() + pos1.z())/0.003;
    
    std::cout << "\nMean angle error[°]: \n";
    std::cout << (std::abs(toDeg(roll)) + std::abs(toDeg(pitch)) + std::abs(toDeg(yaw)))/3 << "\n";
    
    //printf("\nX: %f \nY: %f\nZ: %f\n", std::abs(x), y, z);
    //printf("\nx: %f \ny: %f\nz: %f \nw: %f\n", std::abs(rx), ry, rz, rw);
    
    //std::cout << "\nworld to recalc_world\n";
    //printf("\nX: %f \nY: %f\nZ: %f\n", pos2.x(), pos2.y(), pos2.z());
    //printf("\nx: %f \ny: %f\nz: %f \nw: %f\n", rot2.x(), rot2.y(), rot2.z(), rot2.w());
            
    rate.sleep();
  }
  return 0;
};

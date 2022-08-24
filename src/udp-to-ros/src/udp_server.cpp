#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/PointStamped.h"

using namespace std;
   
#define PORT    5000 
#define MAXLINE 1024 

ros::Publisher positionPublisher;
uint32_t sequenceNumber = 0;

std::string int2str(int x)
{
    std::stringstream ss; // create a stringstream
    ss << x;              // add number to the stream
    return ss.str();      // return stream content
}

void poseCallback(int id, float x, float y, float z, float rX, float rY, float rZ, tf::Matrix3x3 r) {
    if(id == 1) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x/1000, y/1000, z/1000) );
        
        static tf::TransformBroadcaster br1;
        tf::Transform artWorld;
        //ex1 15000 samples
        //artWorld.setOrigin( tf::Vector3(-0.150, -0.283, -0.082) );
        //artWorld.setRotation(tf::Quaternion(0,0,0,1));
        
        //ex2 4500 samples
        //artWorld.setOrigin( tf::Vector3(-0.046, -0.3, -0.065) );
        //artWorld.setRotation(tf::Quaternion(-0.0060,0.0035, -0.0125,0.9987));
        
        //ex3 4500 samples
        //artWorld.setOrigin( tf::Vector3(0.084, -0.244, -0.092) );
        //artWorld.setRotation(tf::Quaternion(0.0005, -0.0040, -0.0267,0.9987));
        
        
        //ex4 9000 samples static
        //artWorld.setOrigin( tf::Vector3(-0.060, -0.302, -0.083) );
        //artWorld.setRotation(tf::Quaternion(-0.0177, 0.0112, -0.0113,0.9996));
        
        //ex5 1000 samples static
        artWorld.setOrigin( tf::Vector3(0.05939775, -0.19979336, -0.14470179) );
        artWorld.setRotation(tf::Quaternion(-0.00426966, -0.01274455, -0.02240905,  0.99962411));
        
        br1.sendTransform(tf::StampedTransform(artWorld, ros::Time::now(), "world", "ART_world" ));
        
        tf::Quaternion q;
        r.getRotation(q);
        
        q = q.inverse();
        transform.setRotation(q);
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ART_world", "ART_pen" ));
        transform = transform.inverse();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "markers", "recalc_world" ));

        geometry_msgs::PointStamped markerPosition;
        markerPosition.header.seq = sequenceNumber++;
        markerPosition.header.stamp = ros::Time::now();
        markerPosition.header.frame_id = "map";
        markerPosition.point.x = x / 1000.0f;
        markerPosition.point.y = y / 1000.0f;
        markerPosition.point.z = z / 1000.0f;
        positionPublisher.publish(markerPosition);
    }
}
  
int stoi(string s) {
    stringstream intValue(s);
    int number = 0;
    intValue >> number;
    return number;
}

float stof(string s) {
    float number;
    std::stringstream ss;
    ss << s;
    ss >> number;
    return number;
}

double Convert(double degree) 
{ 
    double pi = 3.14159265359; 
    return (degree * (pi / 180)); 
} 

bool messageParser(char* buffer) {
    string sBuffer = string(buffer);
    cout << buffer;
    int header = sBuffer.find("6d");
    
    int frameId = stoi(sBuffer.substr(3, header));
    
    int numberOfDevices = stoi(sBuffer.substr(header + 3, header + 4));
   
    printf("\nFrame Id: %d \nNumer Of Devices: %d\n", frameId, numberOfDevices);
        
    if(numberOfDevices != 0) {
        for(int i =0; i < numberOfDevices; i++) {
            
            sBuffer.erase(0, sBuffer.find("["));
            
            //process first []
            int deviceId = stoi(sBuffer.substr(1, sBuffer.find(" "))) + 1;
            sBuffer.erase(0, sBuffer.find("]") + 1);

            //process second []
                
            //translation
            float x = stof(sBuffer.substr(1, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float y = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float z = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            //rotation
            float rX = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float rY = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float rZ = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find("]") + 1);
          
            //process third []
                
            float xx = stof(sBuffer.substr(1, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float xy = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float xz = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float yx = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float yy = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float yz = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float zx = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float zy = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find(" ") + 1);

            float zz = stof(sBuffer.substr(0, sBuffer.find(" ")));
            sBuffer.erase(0, sBuffer.find("]") + 1);

            tf::Matrix3x3 r(xx,xy,xz,yx,yy,yz,zx,zy,zz);

            printf("\nDevice ID: %d\n", deviceId);
            printf("\nX: %f \nY: %f\nZ: %f\n", x, y, z);
            printf("\nrX[°]: %f \nrY[°]: %f\nrZ[°]: %f\n", rX, rY, rZ);
            printf("\nRotation Matrix:\n");
            cout << xx << " " << xy << " " << xz << "\n";
            cout << yx << " " << yy << " " << yz << "\n"; 
            cout << zx << " " << zy << " " << zz << "\n"; 

            poseCallback(deviceId, x, y, z, rX, rY, rZ, r);
        }
    }
}

// Driver code 
int main(int argc, char** argv) { 
    ros::init(argc, argv, "my_tf_broadcaster");

    ros::NodeHandle nodeHandle = ros::NodeHandle();
    positionPublisher = nodeHandle.advertise<geometry_msgs::PointStamped>("trackingSystemMarkerPosition", 10);
        
    int sockfd; 
    char buffer[MAXLINE]; 
    char *hello = "Hello from server"; 
    struct sockaddr_in servaddr, cliaddr; 
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    unsigned int len, n; 
  
    len = sizeof(cliaddr);  //len is value/resuslt 
    while(ros::ok()) {
        n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
                    &len); 
        buffer[n] = '\0';
        messageParser(buffer);
    }
      
    return 0; 
}

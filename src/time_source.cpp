#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>

#include <sstream>

image_transport::Publisher pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "time_source");
  ros::NodeHandle n("~");

  ros::Publisher pub_trajectory = n.advertise<visualization_msgs::Marker>("time", 100);
  ros::Rate loop_rate(10);

  int count = 0;
  while(ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time().fromSec(count);
    pub_trajectory.publish(marker);
    std::cout<<"time "<<count<<std::endl;

    if(count%10==0) std::cout<<count<<" "<<std::setprecision(15)<<ros::Time::now().toSec()<<std::endl;

    loop_rate.sleep();
    count++;
  }
  return 0;
}
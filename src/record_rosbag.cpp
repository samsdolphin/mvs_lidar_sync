#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "CustomMsg.h"
#include <ros/ros.h>

bool enable_compressed = true, avia_init = false, exit_flag = false;

void PressEnterToExit(void)
{
  int c;
  while((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress enter to exit.\n");
  while(getchar() != '\n')
    ;
  exit_flag = true;
  sleep(1);
}

void cam_callback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!avia_init)
  {
    // ros::Duration(0.09).sleep(); // make sure camera exposure
    std::string path = "/home/sam/"+std::to_string(msg->header.stamp.toSec())+".bag";
    std::string topics;
    if(enable_compressed)
      topics = " /right_camera/image/compressed /ouster/points";
    else
      topics = " /right_camera/image /ouster/points";
    
    std::string node_name = " __name:=my_record_node";
    std::string cmd_str = "gnome-terminal -x bash -c 'rosbag record -O " + path + topics + node_name + "'";
    bool ret = system(cmd_str.c_str()); // #include <stdlib.h>
    avia_init = true;
    ROS_DEBUG("Camera Init!");
  }
}

void avia_callback(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
  if(!avia_init)
  {
    std::string path = "/home/sam/"+std::to_string(msg->header.stamp.toSec())+".bag";
    std::string topics;
    if(enable_compressed)
      // topics = " /highres/image/compressed /livox/imu /livox/lidar";
      topics = " /left_camera/image/compressed /right_camera/image/compressed /livox/imu /livox/lidar";
    else
      topics = " /left_camera/image /right_camera/image /livox/imu /livox/lidar";
    
    std::string node_name = " __name:=my_record_node";
    std::string cmd_str = "'rosbag record -O " + path + topics + node_name + "'";
    // std::string cmd_str = "gnome-terminal -x bash -c 'rosbag record -O " + path + topics + node_name + "'";
    bool ret = system(cmd_str.c_str()); // #include <stdlib.h>
    avia_init = true;
  }
  PressEnterToExit();
}

void ouster_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if(!avia_init)
  {
    std::string path = "/home/sam/"+std::to_string(msg->header.stamp.toSec())+".bag";
    std::string topics;
    if(enable_compressed)
      topics = " /right_camera/image/compressed /ouster/points";
    else
      topics = " /right_camera/image /ouster/points";
    
    std::string node_name = " __name:=my_record_node";
    // std::string cmd_str = "'rosbag record -O " + path + topics + node_name + "'";
    std::string cmd_str = "gnome-terminal -x bash -c 'rosbag record -O " + path + topics + node_name + "'";
    bool ret = system(cmd_str.c_str()); // #include <stdlib.h>
    avia_init = true;
  }
  PressEnterToExit();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "record_rosbag");
  ros::NodeHandle nh("~");

  nh.getParam("enable_compressed", enable_compressed);

  ros::Subscriber sub_surf = nh.subscribe("/ouster/points", 1000, ouster_callback);
  // ros::Subscriber sub_surf = nh.subscribe("/livox/lidar", 1000, avia_callback);
  // ros::Subscriber sub_surf = nh.subscribe("/right_camera/image", 1000, cam_callback);
  
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    
    if(exit_flag)
    {
      break;
    }
  }

  ros::V_string v_nodes;
  ros::master::getNodes(v_nodes);
  std::string node_name = std::string("/my_record_node");
  auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name.c_str());
  if(it != v_nodes.end())
  {
    std::string cmd_str = "rosnode kill " + node_name;
    bool ret = system(cmd_str.c_str());
    std::cout << "## stop rosbag record cmd: " << cmd_str << std::endl;
  }
}

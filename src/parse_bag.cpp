#include <iostream>
#include <fstream>
#include <string>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/CompressedImage.h"
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;
using namespace Eigen;
double secs_init, nsecs_init, last_time;

string bag_path, write_path, topic_name;


void parse_time(string path, bool is_img_topic = true) 
{
  int msg_cnt = 0;
  bool is_init = false;
  fstream file_;
  file_.open(bag_path, ios::in);
  if(!file_)
  {
    cout << "File " << bag_path << " does not exit" << endl;
    return;
  }
  
  rosbag::Bag bag;
  try
  {
    ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
    bag.open(bag_path, rosbag::bagmode::Read);
    ROS_INFO("Bag %s opened", bag_path.c_str());
  }
  catch (rosbag::BagException e)
  {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return;
  }

  ofstream file_dt, file_t;
  vector<string> topics;
  if(is_img_topic)
  {
    topics.push_back(topic_name);
    file_dt.open(path+"img_inc.csv", std::ofstream::trunc);
    file_t.open(path+"img_t.csv", std::ofstream::trunc);
  }
  else
  {
    topics.push_back(string("/ouster/points"));
    file_dt.open(path+"lidar_inc.csv", std::ofstream::trunc);
    file_t.open(path+"lidar_t.csv", std::ofstream::trunc);
  }
  file_t << "message count,message header time\n";

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for(const rosbag::MessageInstance& m : view)
  {
    double secs, nsecs, inc;
    if(is_img_topic)
    {
      // auto msg = *(m.instantiate<sensor_msgs::Image>());
      auto msg = *(m.instantiate<sensor_msgs::CompressedImage>());
      secs = msg.header.stamp.sec;
      nsecs = msg.header.stamp.nsec;
    }
    else
    {
      auto msg = *(m.instantiate<sensor_msgs::PointCloud2>());
      secs = msg.header.stamp.sec;
      nsecs = msg.header.stamp.nsec;
    }
    file_t << msg_cnt << "," << setprecision(12) << secs+nsecs*1e-9 << "\n";
    msg_cnt++;

    if(!is_init)
    {
      secs_init = secs;
      nsecs_init = nsecs;
      is_init = true;
      secs -= secs_init;
      nsecs -= nsecs_init;
      last_time = secs + nsecs * 1e-9;
    }
    else
    {
      secs -= secs_init;
      nsecs -= nsecs_init;
      inc = secs + nsecs * 1e-9 - last_time;
      last_time = secs + nsecs * 1e-9;
      file_dt << inc * 1e3 << "\n";
    }
  }
  file_dt.close();
  cout << "complete" << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parse_rosbag");
  ros::NodeHandle nh("~");

  nh.getParam("bag_path", bag_path);
  nh.getParam("write_path", write_path);
  nh.getParam("topic_name", topic_name);
  
  parse_time(write_path, true);
  parse_time(write_path, false);
}
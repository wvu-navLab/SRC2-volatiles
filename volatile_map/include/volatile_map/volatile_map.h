#ifndef VOLATILE_MAPPER_H
#define VOLATILE_MAPPER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include "srcp2_msgs/VolSensorMsg.h"
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <std_msgs/Int8.h>
#include <volatile_map/Volatile.h>
#include <volatile_map/VolatileMap.h>

class VolatileMapper
{
public:
  VolatileMapper(ros::NodeHandle &, int num_scouts);

private:
  ros::NodeHandle & nh_;
  volatile_map::VolatileMap VolatileMap_;

  std::vector<ros::Subscriber> volSubs_;
  std::vector<ros::Publisher> stopScoutPub_;

  void volatileSensorCallBack_(const ros::MessageEvent<srcp2_msgs::VolSensorMsg const>& event);


  ros::Publisher volMapPub_;

  // used for timers to determine time since last have seen a vol
  std::vector<ros::Time> lastVolRecordedPerID_;
  double timeOut_;
  double distanceThresh_;





};



#endif

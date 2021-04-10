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
#include <volatile_map/Volatile.h>
#include <volatile_map/VolatileMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <numeric>

class VolatileMapper
{
public:
  VolatileMapper(ros::NodeHandle &, int num_scouts);

private:
  ros::NodeHandle & nh_;
  volatile_map::VolatileMap VolatileMap_;
  nav_msgs::OccupancyGrid VolatileOccupancyGrid_;

  std::vector<ros::Subscriber> volSubs_;

  void volatileSensorCallBack_(const ros::MessageEvent<srcp2_msgs::VolSensorMsg const>& event);

  // void volatileOccupancyGridCallBack_(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  ros::Publisher volMapPub_;

  ros::Publisher volOccGridPub_;



};



#endif

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
#include <std_msgs/Int64.h>
#include <volatile_map/Volatile.h>
#include <volatile_map/VolatileMap.h>
#include <volatile_map/MarkCollected.h>
#include <volatile_map/MarkAssigned.h>
#include <sensor_fusion/GetTruePose.h>

class VolatileMapper
{
public:
  VolatileMapper(ros::NodeHandle &, int num_scouts);
  void Publish();

private:
  ros::NodeHandle & nh_;
  volatile_map::VolatileMap VolatileMap_;
  int num_vols_, num_collect_, num_attempt_ ,num_vols_scout_1_, num_vols_scout_2_;
  std::vector<ros::Subscriber> volSubs_;
  std::vector<ros::Publisher> stopScoutPub_;

  void volatileSensorCallBack_(const ros::MessageEvent<srcp2_msgs::VolSensorMsg const>& event);
  bool markCollected_(volatile_map::MarkCollected::Request &req, volatile_map::MarkCollected::Response &res);
  bool markAssigned_(volatile_map::MarkAssigned::Request &req, volatile_map::MarkAssigned::Response &res);
  void GetTruePose(int scout_id);
  ros::Publisher volMapPub_;

  // used for timers to determine time since last have seen a vol
  std::vector<ros::Time> lastVolRecordedPerID_;
  double timeOut_;
  double distanceThresh_;
  double eps_;


  ros::ServiceServer markCollectedServer_, markAssignedServer_;
  ros::ServiceClient clt_sf_true_pose_sc1,clt_sf_true_pose_sc2;






};



#endif

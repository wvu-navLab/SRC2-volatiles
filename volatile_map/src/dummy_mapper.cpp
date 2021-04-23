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

int main(int argc, char **argv) {
  ros::init(argc, argv, "dummy_mapper");

  ros::NodeHandle nh("");

  //create publisher
  ros::Publisher pub = nh.advertise<volatile_map::VolatileMap>("/volatile_map", 1);

  //params for generating map
  float xmin = -50;
  float xmax =  50;
  float ymin = -50;
  float ymax =  50;

  //create dummy volatile map
  volatile_map::VolatileMap VolatileMapMsg;
  for(int i=0; i<20; i++) {
    volatile_map::Volatile vol;

    //position
    geometry_msgs::PointStamped position;

    //time stamp
    position.header.stamp = ros::Time::now();
    
    //point
    float randx = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float randy = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    position.point.x = xmin + randx*(xmax - xmin);
    position.point.y = ymin + randy*(ymax - ymin);
    position.point.z = 0;
    vol.position = position;

    //add to map
    VolatileMapMsg.vol.push_back(vol);
  }

  while (ros::ok()) {
    pub.publish(VolatileMapMsg);
    ros::spinOnce();
  }

  return 0;
}
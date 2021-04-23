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
#include <gazebo_msgs/ModelStates.h>
#include <string>
#include <vector>

//volatile map variable
volatile_map::VolatileMap map;

// split string based on delimiter
std::vector<std::string> split (std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

//callback for gazebo states (which includes volatiles)
void callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  //clear map
  map.vol.clear();

  //extract volatiles from gazebo states
  int nNames = msg->name.size();

  for(int i=0; i<nNames; i++) {
    //parse string
    std::vector<std::string> words = split(msg->name[i], "-");

    //volatiles are of the form "volatile-<type>-...", so skip if at least 2 
    //words are not parsed
    if(words.size() < 2) {
      continue;
    }

    //check if state is volatile
    bool not_volatile = words[0].compare("volatile");
    if(!not_volatile) {
      volatile_map::Volatile vol;
      
      //volatile type
      vol.type = words[1];

      //volatile position
      geometry_msgs::PointStamped position;
      position.point = msg->pose[i].position;
      vol.position = position;

      //add volatile to map
      map.vol.push_back(vol);
    }
  }

  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "volatile_truth");

  ros::NodeHandle nh("");

  //create publisher
  ros::Publisher pub = nh.advertise<volatile_map::VolatileMap>("/volatile_map", 1);

  //create subscriber
  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, callback);

  while (ros::ok()) {
    pub.publish(map);
    ros::spinOnce();
  }

  return 0;
}
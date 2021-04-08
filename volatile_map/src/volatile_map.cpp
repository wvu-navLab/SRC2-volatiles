#include "volatile_map/volatile_map.h"



VolatileMapper::VolatileMapper(ros::NodeHandle &nh, int num_scouts) : nh_(nh) {

for (int i=0; i<num_scouts; i++) {
   std::string topic;
   topic = "/small_scout_" + std::to_string(i+1) + "/volatile_sensor";
   volSubs_.push_back(nh_.subscribe(topic, 1, &VolatileMapper::volatileSensorCallBack_, this));


  }


  volMapPub_ = nh_.advertise<volatile_map::VolatileMap>("/volatile_map", 1);
}



void VolatileMapper::volatileSensorCallBack_(const ros::MessageEvent<srcp2_msgs::VolSensorMsg const>& event){

  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  char robot_number = topic.c_str()[13];
  std::cout << "debug " << robot_number << std::endl;

  const srcp2_msgs::VolSensorMsg::ConstPtr& msg = event.getMessage();

  volatile_map::Volatile vol;

  vol.type = msg->vol_type;
  vol.distance_to = msg->distance_to;
  vol.smoothed = false;
  vol.collected = false;
  vol.scout_id = std::atoi(&robot_number);


  std::cout << vol.type << " " << vol.distance_to << " "  << vol.scout_id << std::endl;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped T_s2o; // transform sensor in obdo
  try {
    std::string odom_frame;
    std::string volSensor_frame;
    odom_frame = "small_scout_" + std::to_string(std::atoi(&robot_number)) + "_odom";
    volSensor_frame = "small_scout_" + std::to_string(std::atoi(&robot_number)) + "_volatile_sensor";
    T_s2o = tfBuffer.lookupTransform(odom_frame, volSensor_frame,
                                     ros::Time(0), ros::Duration(1.0));
    geometry_msgs::PointStamped current_position;

    current_position.point.x = T_s2o.transform.translation.x;
    current_position.point.y = T_s2o.transform.translation.y;
    current_position.point.z = T_s2o.transform.translation.z;

    current_position.header.stamp=ros::Time::now();

    vol.position = current_position;
  } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }


    if(vol.distance_to > 0.0){

      VolatileMap_.vol.push_back(vol);

      volMapPub_.publish(VolatileMap_);
    }
    // TODO  handle saving case where didnt see a volatile
    //  ROS occupancy grid



}



///////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
ros::init(argc, argv, "volatile_map");

ros::NodeHandle nh("");

ROS_INFO(" Volatile Mapper Node Has Started");
int num_scouts;
nh.getParam("/num_scouts",num_scouts);

VolatileMapper mapper(nh, num_scouts);


while (ros::ok()) {
ros::spinOnce();
// could pub vol map a pre-defined rate
}

return 0;
}

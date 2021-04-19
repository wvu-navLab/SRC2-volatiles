#include "volatile_map/volatile_map.h"



VolatileMapper::VolatileMapper(ros::NodeHandle &nh, int num_scouts) : nh_(nh) {

  timeOut_=0.5;
  distanceThresh_=10;

for (int i=0; i<num_scouts; i++) {
   std::string topic_sub;
   std::string topic_pub;
   topic_sub = "/small_scout_" + std::to_string(i+1) + "/volatile_sensor";
   topic_pub = "/small_scout_" + std::to_string(i+1) + "/volatile_map/stop";
   volSubs_.push_back(nh_.subscribe(topic_sub, 1, &VolatileMapper::volatileSensorCallBack_, this));
   lastVolRecordedPerID_.push_back(ros::Time::now());
   stopScoutPub_.push_back(nh_.advertise<std_msgs::Bool>(topic_pub,1);

  }


  volMapPub_ = nh_.advertise<volatile_map::VolatileMap>("/volatile_map", 1);
}



void VolatileMapper::volatileSensorCallBack_(const ros::MessageEvent<srcp2_msgs::VolSensorMsg const>& event){


  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  char robot_number = topic.c_str()[13];
  std::cout << "debug " << robot_number << std::endl;

  const srcp2_msgs::VolSensorMsg::ConstPtr& msg = event.getMessage();
  //if this is not a valid volatile (i.e., ==-1), just perform quick return
  if( msg->distance_to == -1 ){
    return;
  }

  volatile_map::Volatile vol;

  vol.type = msg->vol_type;
  vol.distance_to = msg->distance_to;
  vol.smoothed = false;
  vol.collected = false;
  vol.failed_to_collect = false;
  vol.attempted = false;
  vol.honed = false;
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

    bool haveSeenThisVol= false;
    int index = -1;
    for(std::vector<Volatile>::iterator it = VolatileMap_.begin(); it != VolatileMap_.end(); ++it) {
      double dx = vol.position.x - it->position.x;
      double dy = vol.position.y - it->position.y;
      double distance = pow(dx*dx + dy*dy), 0.5);
      // if we are close to where we saw this volatile and it is same type, assume it is the same one
      // TODO TEST THESE THRESH
      if( distance < distanceThresh_ && (vol.type == it-> type) ){
        haveSeenThisVol = true;
        index = it-VolatileMap_.begin();

      }

    }

    // if we have never seen this volatile, we push back and publish the map
    if(!haveSeenThisVol){
      VolatileMap_.vol.push_back(vol);
      volMapPub_.publish(VolatileMap_);
    }
    else{
      // we have seen this volatile before.
      // was it recently?
      ros::Duration deltaT = ros::Time::now() - lastVolRecordedPerID_[vol.scout_id];
      // TODO TEST THIS TIMEOUT THRESH
      if(deltaT < timeOut_){

          // are we closer now?
          if(vol.distance_to < VolatileMap_[index].distance_to){
            // if this is a continuous track and we are still getting closer,
            // replace the volatile and publish map again
            VolatileMap_[index] = vol;
            // republish map with updated location
            volMapPub_.publish(VolatileMap_);

          }
          // continuous tracking, but now we are farther away than we were
          else{


            // publish flag to tell state machine to STOP
            // only if we are aleady trying this honeing manuever
            // dont keep this volatile and dont publish map
            if(!VolatileMap_[index].honed){
              VolatileMap_[index].honed=true;
              std_msgs::Bool stop_msg;
              stop.data= true;
              stopScoutPub_[vol.scout_id].publish(stop);
          }
          }


      }
      else{
        // this means we are not continuously tracking this vol, but we have seen it before
        // TODO we need to have some logic to determine what is to do, based on occupancy map
        // for now, only replace if it has been marked as a failed_to_collect volatile
          if(VolatileMap_[index].failed_to_collect){
            VolatileMap_[index] = vol;
            volMapPub_.publish(VolatileMap_);
          }

       }



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

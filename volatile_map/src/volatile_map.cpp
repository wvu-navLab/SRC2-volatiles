#include "volatile_map/volatile_map.h"



VolatileMapper::VolatileMapper(ros::NodeHandle &nh, int num_scouts)
  : nh_(nh)
{
  timeOut_=0.5;
  distanceThresh_=8;
  eps_ = 0.001;
  num_vols_=0;
  num_collect_ =0;
  num_attempt_ =0;
  num_vols_scout_1_=0;
  num_vols_scout_2_=0;

  for (int i=0; i<num_scouts; i++)
  {
    std::string topic_sub;
    std::string topic_pub;
    topic_sub = "/small_scout_" + std::to_string(i+1) + "/volatile_sensor";
    topic_pub = "/small_scout_" + std::to_string(i+1) + "/volatile_map/cmd";
    volSubs_.push_back(nh_.subscribe(topic_sub, 1, &VolatileMapper::volatileSensorCallBack_, this));
    lastVolRecordedPerID_.push_back(ros::Time::now());
    stopScoutPub_.push_back(nh_.advertise<volatile_map::VolCmd>(topic_pub,1));
  }
  volMapPub_ = nh_.advertise<volatile_map::VolatileMap>("/volatile_map", 1);

  markCollectedServer_ = nh_.advertiseService("/volatile_map/mark_collected",&VolatileMapper::markCollected_,this);
  markAssignedServer_ = nh_.advertiseService("/volatile_map/mark_assigned",&VolatileMapper::markAssigned_,this);
  markHonedServer_ = nh_.advertiseService("/volatile_map/mark_honed",&VolatileMapper::markHoned_,this);

    clt_sf_true_pose_sc1 = nh.serviceClient<sensor_fusion::GetTruePose>("/small_scout_1/true_pose");
    clt_sf_true_pose_sc2 = nh.serviceClient<sensor_fusion::GetTruePose>("/small_scout_2/true_pose");
}

bool VolatileMapper::markCollected_(volatile_map::MarkCollected::Request &req, volatile_map::MarkCollected::Response &res){
  int volIndex = req.vol_index;
  bool collected = req.collected;
  bool attempted = req.attempted;
  if(collected) num_collect_ = num_collect_ +1;
  if(attempted) num_attempt_ = num_attempt_ +1;
  for (int i=0; i< VolatileMap_.vol.size(); i++)
  {
    if(volIndex == VolatileMap_.vol[i].vol_index)
    {
    VolatileMap_.vol[i].collected = collected;
    VolatileMap_.vol[i].attempted = attempted;
    ROS_WARN_STREAM("VolMapper: Marking Vol" << volIndex << " Collected: " << collected << " Attempted:" << attempted );
    }
  }

  volMapPub_.publish(VolatileMap_);
  res.success=true;

	return true;
}

bool VolatileMapper::markHoned_(volatile_map::MarkHoned::Request &req, volatile_map::MarkHoned::Response &res){
  int volIndex = req.vol_index;
  bool honed = req.honed;
  for (int i=0; i< VolatileMap_.vol.size(); i++)
  {
    if(volIndex == VolatileMap_.vol[i].vol_index)
    {
    VolatileMap_.vol[i].honed = honed;
    ROS_WARN_STREAM("VolMapper: Marking Vol" << volIndex << " Honed: " << honed );
    }
  }

  volMapPub_.publish(VolatileMap_);
  res.success=true;

	return true;
}


bool VolatileMapper::markAssigned_(volatile_map::MarkAssigned::Request &req, volatile_map::MarkAssigned::Response &res){
  int robot_id = req.robot_id_assigned;
  int volIndex = req.vol_index;


  for (int i=0; i< VolatileMap_.vol.size(); i++)
  {
    if(volIndex == VolatileMap_.vol[i].vol_index)
    {
    VolatileMap_.vol[i].robot_id_assigned = robot_id;

    ROS_WARN_STREAM("VolMapper: Marking Vol" << volIndex << " Assigned to: " << robot_id );
    }
  }

  volMapPub_.publish(VolatileMap_);
  res.success=true;

	return true;
}


void VolatileMapper::Publish(){
  volMapPub_.publish(VolatileMap_);
  ROS_INFO_STREAM(" !!Volatile Mapper!! # Vols Mapped -->" << num_vols_ << " # Attemped -->" << num_attempt_ << " Collected -->" << num_collect_);
}

void VolatileMapper::GetTruePose(int scout_id)
{
  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  srv_sf_true_pose.request.initialize = false;

  if(scout_id == 1)
  {
    if (clt_sf_true_pose_sc1.call(srv_sf_true_pose))
    {
      ROS_INFO_STREAM("[Vol Map Scout 1] Called service TruePose");

    }
    else
    {
      ROS_INFO_STREAM("[Vol Map Scout 1] Failed to Call service TruePose");
    }
  }

  if(scout_id == 2)
  {
    if (clt_sf_true_pose_sc2.call(srv_sf_true_pose))
    {
      ROS_INFO_STREAM("[Vol Map Scout 2] Called service TruePose");

    }
    else
    {
      ROS_INFO_STREAM("[Vol Map Scout 2] Failed to Call service TruePose");
    }
  }


}


void VolatileMapper::volatileSensorCallBack_(const ros::MessageEvent<srcp2_msgs::VolSensorMsg const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  char robot_number = topic.c_str()[13];
  // std::cout << "debug " << robot_number << std::endl;

  const srcp2_msgs::VolSensorMsg::ConstPtr& msg = event.getMessage();

  //if this is not a valid volatile (i.e., ==-1), just perform quick return
  if( msg->distance_to == -1 )
  {
    return;
  }

  volatile_map::Volatile vol;

  vol.type = msg->vol_type;
  vol.distance_to = msg->distance_to;
  vol.wellKnown = false;
  vol.collected = false;
  vol.vol_index = 0;
  vol.failed_to_collect = false;
  vol.attempted = false;
  vol.honing = false;
  vol.honed = false;
  vol.slow = false;
  vol.robot_id_assigned =0;



  vol.scout_id = std::atoi(&robot_number);
  lastVolRecordedPerID_[vol.scout_id-1]=ros::Time::now();
  std::cout << vol.type << " " << vol.distance_to << " "  << vol.scout_id << std::endl;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped T_s2o; // transform sensor in obdo
  try
  {
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
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  bool haveSeenThisVol= false;
  int index = -1;

  for(std::vector<volatile_map::Volatile>::iterator it = VolatileMap_.vol.begin(); it != VolatileMap_.vol.end(); ++it)
  {
    double dx = vol.position.point.x - it->position.point.x;
    double dy = vol.position.point.y - it->position.point.y;
    double distance = hypot(dx,dy);
    // if we are close to where we saw this volatile and it is same type, assume it is the same one
    // TODO TEST THESE THRESH
    if(distance < distanceThresh_ && (vol.type == it-> type))
    {
      // std::cout << "WE HAVE SEEN THIS VOL and  " << distance << std::endl;
      haveSeenThisVol = true;
      index = it-VolatileMap_.vol.begin();
    }
  }

  // if we have never seen this volatile, we push back
  if(!haveSeenThisVol)
  {
    if(!vol.slow){
      vol.slow=true;
      // std::cout << " Have Not Seen this Vol  and Publish Slow " << std::endl;
      volatile_map::VolCmd cmd_msg;
      cmd_msg.cmd= 1;
      cmd_msg.vol_index = vol.vol_index;
      stopScoutPub_[vol.scout_id-1].publish(cmd_msg);
    }
    vol.vol_index = num_vols_;
    VolatileMap_.vol.push_back(vol);
    num_vols_ = num_vols_+1;

    if(vol.scout_id == 1)
    {
      num_vols_scout_1_ = num_vols_scout_1_ +1;
    }
    if(vol.scout_id == 2)
    {
      num_vols_scout_2_ = num_vols_scout_2_ +1;
    }
    if(num_vols_scout_1_ == 2)
    {
      GetTruePose(1);
    }

    if(num_vols_scout_1_ == 2)
    {
      GetTruePose(2);
    }

  //  volMapPub_.publish(VolatileMap_);

  }
  else
  {

    // we have seen this volatile before.
    // was it recently?
    ros::Duration deltaT = ros::Time::now() - lastVolRecordedPerID_[vol.scout_id-1];
    // TODO TEST THIS TIMEOUT THRESH
    if(deltaT.toSec() < timeOut_)
    {
      // are we closer now?
      if(vol.distance_to <= VolatileMap_.vol[index].distance_to +eps_)
      {
        // if this is a continuous track and we are still getting closer,
        // replace the volatile and publish map again
        vol.slow=true;
        vol.vol_index=VolatileMap_.vol[index].vol_index;
        vol.collected=VolatileMap_.vol[index].collected;
        vol.attempted=VolatileMap_.vol[index].attempted;
        vol.honing=VolatileMap_.vol[index].honing;
        vol.honed=VolatileMap_.vol[index].honed;
        vol.robot_id_assigned=VolatileMap_.vol[index].robot_id_assigned;
        VolatileMap_.vol[index] = vol;
        //volMapPub_.publish(VolatileMap_);

      }
      // continuous tracking, but now we are farther away than we were
      else
      {

        // publish flag to tell state machine to STOP
        // if we already told to slow, but have not triggered honing
        // dont keep this volatile and dont publish map
        if(!VolatileMap_.vol[index].honing && VolatileMap_.vol[index].slow)
        {
          VolatileMap_.vol[index].honing=true;
          volatile_map::VolCmd cmd_msg;
          cmd_msg.cmd = 2;
          cmd_msg.vol_index = VolatileMap_.vol[index].vol_index;
          // std::cout << " Publishing Stop " << vol.distance_to << " " << VolatileMap_.vol[index].distance_to <<std::endl;
          stopScoutPub_[vol.scout_id-1].publish(cmd_msg);
        }
        else if(VolatileMap_.vol[index].honing && VolatileMap_.vol[index].slow )
        {
          // this condition means that we have triggered a slow to SM when first seeing
          // we have also trigger honing, and now we are further away
          // this means we are honed
      //    VolatileMap_.vol[index].honed = true;
        }

        // publish the volatile map with the closest location we found
        volMapPub_.publish(VolatileMap_);

      }
    }
    else
    {
      // this means we are not continuously tracking this vol, but we have seen it before
      // TODO we need to have some logic to determine what is to do, based on occupancy map
      // for now, only replace if it has been marked as a failed_to_collect volatile
      if(VolatileMap_.vol[index].failed_to_collect)
      {
        vol.vol_index=VolatileMap_.vol[index].vol_index;
        vol.collected=VolatileMap_.vol[index].collected;
        vol.attempted=VolatileMap_.vol[index].attempted;
        vol.honing=VolatileMap_.vol[index].honing;
        vol.honed=VolatileMap_.vol[index].honed;
        vol.robot_id_assigned=VolatileMap_.vol[index].robot_id_assigned;
        VolatileMap_.vol[index] = vol;
        volMapPub_.publish(VolatileMap_);
      }
    }
  }
    // TODO  handle saving case where didnt see a volatile
    //  ROS occupancy grid
}



///////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "volatile_map");

  ros::NodeHandle nh("");

  ROS_INFO(" Volatile Mapper Node Has Started");
  int num_scouts;
  nh.getParam("/num_scouts",num_scouts);

  VolatileMapper mapper(nh, num_scouts);

  ros::Rate rate(10);
  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    count = count +1;
    if(count == 10)
    {
      mapper.Publish();
      count = 0;

    }
    // could pub vol map a pre-defined rate
  }

  return 0;
}

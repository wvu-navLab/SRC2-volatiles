#include "volatile_map/volatile_map.h"
VolatileMapper::VolatileMapper(ros::NodeHandle &nh) : nh_(nh) {
}



int main(int argc, char **argv) {
ros::init(argc, argv, "volatile_mapper");

ros::NodeHandle nh("");

ROS_INFO(" Volatile Mapper Node Has Started");

VolatileMapper mapper(nh);


while (ros::ok()) {
ros::spinOnce();
}

return 0;
}

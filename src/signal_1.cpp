#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::NodeHandle m;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher signal = m.advertise<geometry_msgs::Pose>("signal",1000);

  geometry_msgs::Pose pomiar;
  pomiar.position.x = 1;
  pomiar.position.y = 1;
  pomiar.position.z = 1;
  pomiar.orientation.w = 0;
  pomiar.orientation.x = 1;
  pomiar.orientation.y = 1;
  pomiar.orientation.z = 1;
  

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    signal.publish(pomiar);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
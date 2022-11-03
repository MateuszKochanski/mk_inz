#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/String.h"

void chatterCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
  fprintf(stdout,"Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n",
    msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z);
}

void actualTCPposition(const geometry_msgs::Wrench::ConstPtr& msg)
{
  fprintf(stdout,"Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n",
    msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber hex = n.subscribe("signal", 1000, chatterCallback);
  ros::Subscriber TCP = n.subscribe("tcp", 1000, actualTCPposition);
  ros::Publisher sterowanie = n.advertise<std_msgs::String>("sterowanie",1000);

  std_msgs::String data;
  data.data = "daneeee";
	geometry_msgs::Wrench pomiar;

  ros::Rate loop_rate(100); 
  
  while (ros::ok())
  {  
    ros::spinOnce();
    fprintf(stdout,"costam\n");
    sterowanie.publish(data);
    loop_rate.sleep();
  }

  return 0;
}

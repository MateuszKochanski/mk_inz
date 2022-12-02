#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Publisher sterowanie;
geometry_msgs::Pose actualPose;
geometry_msgs::Pose nextPose;
geometry_msgs::Wrench actualForces;
int loopRate = 100;
double timeWithoutHex;
const double maxTimeWithoutHex = 0.2;
bool iGotActualPose = false;
bool moveIsDone = true;
bool anyForce = false;


double forceValue(geometry_msgs::Wrench hex)
{
  return sqrt(pow(hex.force.x, 2) + pow(hex.force.y, 2) + pow(hex.force.z, 2));
}

bool czySilaPrzekracza(geometry_msgs::Wrench hex, double C)
{
  if (abs(hex.force.x) > C || abs(hex.force.y) > C || abs(hex.force.z) > C)
    return true;
  else
    return true;
}

bool czyJestSila(geometry_msgs::Wrench hex)
{
  if(czySilaPrzekracza(hex, 0.5))
    return true;
  else
    return false;
}


double cutValue(double wartosc, double prog, double skala)
{ 
  double wynik;
  if (wartosc > prog)
    wynik = (wartosc - prog);
  else if (wartosc < -prog)
    wynik = (wartosc + prog);
  else
    wynik = 0;

  return wynik * skala;
}

void findTransformToNextPosition(geometry_msgs::Wrench forces)
{

  static tf::TransformBroadcaster br;
  double progF = 0.5;
  double progT = 0.5;
  double scale = 0.01;
  double aScale = 0.00;
  double x, y, z, rx, ry, rz;

  tf::Quaternion angleTransform;
  
  x = cutValue(forces.force.x, progF, scale);
  y = cutValue(forces.force.y, progF, scale);
  z = cutValue(forces.force.z, progF, scale);

  rx = cutValue(forces.torque.x, progT, aScale);
  ry = cutValue(forces.torque.y, progT, aScale);
  rz = cutValue(forces.torque.z, progT, aScale);

  angleTransform.setRPY(rx, ry, rz);
  // tf::Transform transform(angleTransform, tf::Vector3(x, y, z));
  tf::Transform transform(angleTransform, tf::Vector3(-y, x, z));

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "TCP", "tmp"));
}
/*
bool meetATarget()
{
  double odleglosc = tf::tfDistance(tf::Vector3(actualPose.position.x, actualPose.position.y, actualPose.position.z), tf::Vector3(nextPose.position.x, nextPose.position.y, nextPose.position.z));
  //ROS_INFO("%2f",odleglosc);
  if(odleglosc < 0.01 )
    return true;
  else
    return false;
}*/

bool calculateGlobalTargetPosition()
{
  bool succed = false;

  static tf::TransformListener listener;
  static tf::TransformBroadcaster broadcaster;
  tf::StampedTransform stampedTransform;
  bool getTransform = false;
  
  try
  {
    listener.lookupTransform("base","tmp",ros::Time(0), stampedTransform);
    tf::Transform transform;
    transform.setOrigin(stampedTransform.getOrigin());
    transform.setRotation(stampedTransform.getRotation());
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "NEXT_TCP"));

    // nextPose.orientation.x = transform.getRotation().getX();
    // nextPose.orientation.y = transform.getRotation().getY();
    // nextPose.orientation.z = transform.getRotation().getZ();
    // nextPose.orientation.w = transform.getRotation().getW();
    tf::Matrix3x3 m(transform.getRotation());
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    yaw = yaw*(180.0/3.14);
    pitch = pitch*(180.0/3.14);
    roll = roll*(180.0/3.14);

    nextPose.orientation.x = yaw;
    nextPose.orientation.y = pitch;
    nextPose.orientation.z = roll;
    ROS_INFO("%2f, %2f, %2f", yaw, pitch, roll);

    nextPose.position.x = transform.getOrigin().getX();
    nextPose.position.y = transform.getOrigin().getY();
    nextPose.position.z = transform.getOrigin().getZ();
    //iHaveTarget = true;
    succed = true;

  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  return succed;
}

void resetForces()
{
    actualForces.force.x = 0;
    actualForces.force.y = 0;
    actualForces.force.z = 0;
    actualForces.torque.x = 0;
    actualForces.torque.y = 0;
    actualForces.torque.z = 0;
    anyForce = false;
}

void chatterCallback(const geometry_msgs::Wrench& msg)
{

  static geometry_msgs::Wrench poprzedniHex(msg);
  fprintf(stdout,"Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n",
    msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z);
  if(czyJestSila(msg))
  {
    actualForces = msg;
    anyForce = true;
  }
  else
  { 
    resetForces();
  } 
  timeWithoutHex = 0;
}

void doneCallback(const std_msgs::String::ConstPtr& str)
{
  std::cout<<str->data.c_str()<<std::endl;
  std::string ok = "ok";
  if(str->data.c_str() == ok.c_str())
  {
    
    ROS_INFO("OK");
  }
  moveIsDone = true;
}

tf::Quaternion msgToTfDatatype(geometry_msgs::Quaternion msg)
{
  tf::Quaternion a;
  a.setX(msg.x);
  a.setY(msg.y);
  a.setZ(msg.z);
  a.setW(msg.w);
  return a;
}

void actualTCPposition(const geometry_msgs::Pose& msg)
{
  static tf::TransformBroadcaster br;
  tf::Quaternion a = msgToTfDatatype(msg.orientation);
  tf::Quaternion b = a.inverse();
    
  
  tf::Matrix3x3 n(b);
  double roll1, pitch1, yaw1;
  n.getEulerYPR(yaw1, pitch1, roll1);


  tf::Matrix3x3 m(a);
  double roll, pitch, yaw;
  m.getEulerYPR(yaw, pitch, roll);
  //.getEulerZYX(z, y, x, 2);

  

  //fprintf(stdout,"Position: X:%2f Y:%2f Z:%2f   Rotation: W:%2f X:%2f Y:%2f Z:%2f \r\n",msg.position.x, msg.position.y, msg.position.z, 
  //msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z );
  //fprintf(stdout,"Position1: X:%2f Y:%2f Z:%2f   Rotation: X:%2f Y:%2f Z:%2f \r\n",msg.position.x, msg.position.y, msg.position.z, roll*(180.0/M_PI), pitch*(180.0/M_PI), yaw*(180.0/M_PI));
  //fprintf(stdout,"Position2: X:%2f Y:%2f Z:%2f   Rotation: X:%2f Y:%2f Z:%2f \r\n",msg.position.x, msg.position.y, msg.position.z, roll1*(180.0/M_PI), pitch1*(180.0/M_PI), yaw1*(180.0/M_PI));
  actualPose = msg;
  


  tf::Transform transform(a, tf::Vector3(msg.position.x, msg.position.y, msg.position.z));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "TCP"));

  iGotActualPose = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forceControler");
  ros::NodeHandle n;

  ros::Subscriber hex = n.subscribe("hex", 10, chatterCallback);
  ros::Subscriber TCP = n.subscribe("/es_arm/cartesian_pose", 10, actualTCPposition);
  ros::Subscriber done = n.subscribe("/es_master/done", 10, doneCallback);
  sterowanie = n.advertise<geometry_msgs::Pose>("sterowanie",1000);

	geometry_msgs::Wrench pomiar;

  ros::Rate loop_rate(loopRate); 


  timeWithoutHex = 0;
/////////////////////////////////////////////////////Do zakomentowania
  actualForces.force.x = 0;
  actualForces.force.y = 1;
  actualForces.force.z = 0;
  actualForces.torque.x = 0;
  actualForces.torque.y = 0;
  actualForces.torque.z = 0;
  anyForce = true;
////////////////////////////////////////////////////

  while (ros::ok())
  {  
    /*
    if(timeWithoutHex >= maxTimeWithoutHex)
    {
      ROS_WARN("No connection with HEX!");
      resetForces();
    }
    else
      timeWithoutHex += (1.0/(double(loopRate)));
    */

    if(moveIsDone)
    {
      if(iGotActualPose && anyForce)
      {
        findTransformToNextPosition(actualForces);
        if(calculateGlobalTargetPosition())
        {
          moveIsDone = false; 
          sterowanie.publish(nextPose);
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}

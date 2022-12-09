#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Publisher sterowanie;
geometry_msgs::Pose nextPose;
geometry_msgs::Wrench actualForces;
geometry_msgs::PoseStamped messege;
int loopRate = 100;
bool iHaveActualPose = false;
bool moveIsDone = true;
bool anyForce = false;
double timeWithoutDone;
bool nextPoseAccepted = true;
ros::Time lastActualPoseTime;
ros::Time lastHexTime;

double forceValue(geometry_msgs::Wrench hex)
{
  return sqrt(pow(hex.force.x, 2) + pow(hex.force.y, 2) + pow(hex.force.z, 2));
}

bool czySilaPrzekracza(geometry_msgs::Wrench hex, double C)
{
  if (abs(hex.force.x) > C || abs(hex.force.y) > C || abs(hex.force.z) > C)
    return true;
  else
    return false;
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
  double progT = 0.1;
  double scale = 0.01;
  double aScale = 1.5;
  double x, y, z, rx, ry, rz;

  tf::Quaternion angleTransform;
  
  x = cutValue(forces.force.x, progF, scale);
  y = cutValue(forces.force.y, progF, scale);
  z = cutValue(forces.force.z, 0.7, scale);

  rx = cutValue(forces.torque.x, progT, 0);
  ry = cutValue(forces.torque.y, progT, 0);
  rz = cutValue(forces.torque.z, progT, aScale);

  fprintf(stdout,"Fx: %.2f N Fy: %.2f N Fz: %.2f N Tz: %.2f dx: %.4f m dy: %.4f m dz: %.4f drz: %.4f rad\r\n",
    actualForces.force.x, actualForces.force.y, actualForces.force.z, actualForces.torque.z, x, y, z, rz);

  angleTransform.setRPY(rx, ry, rz);
  // tf::Transform transform(angleTransform, tf::Vector3(x, y, z));
  tf::Transform transform(angleTransform, tf::Vector3(-y, x, z));

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "TCP", "tmp"));
}


bool calculateGlobalTargetPosition()
{
  //geometry_msgs::Pose *nextPosee = nullptr;

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

    //ROS_INFO("%2f", roll);

    //yaw obrot w osi z
    //pitch obrot w y
    //roll obrot w x

    nextPose.orientation.x = yaw; 
    nextPose.orientation.y = pitch;
    nextPose.orientation.z = roll;
    ROS_INFO("%2f, %2f, %2f", yaw, pitch, roll);

    nextPose.position.x = transform.getOrigin().getX();
    nextPose.position.y = transform.getOrigin().getY();
    nextPose.position.z = transform.getOrigin().getZ();
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

  // fprintf(stdout,"Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n",
  //   msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z);
  if(czyJestSila(msg))
  {
    actualForces = msg;
    anyForce = true;
  }
  else
  { 
    resetForces();
  } 
  lastHexTime = ros::Time::now();
  
}

void doneCallback(const std_msgs::Header head)
{
  if(head.stamp == messege.header.stamp)
  {
    if(head.frame_id == "accepted")
      nextPoseAccepted = true;
    else if(head.frame_id == "done")
    {
      moveIsDone = true;
    }
  }
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
  
  tf::Transform transform(a, tf::Vector3(msg.position.x, msg.position.y, msg.position.z));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "TCP"));

  lastActualPoseTime = ros::Time::now();
  iHaveActualPose = true;
}


geometry_msgs::PoseStamped prepareMessege(geometry_msgs::Pose pose)
{
  geometry_msgs::PoseStamped msg;
  msg.pose = pose;

  msg.header.frame_id = "NextPose";
  msg.header.stamp = ros::Time::now();
  //msg.header.seq = 0;

  return msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "forceControler");
  ros::NodeHandle n;

  ros::Subscriber hex = n.subscribe("hex", 10, chatterCallback);
  ros::Subscriber TCP = n.subscribe("/es_arm/cartesian_pose", 10, actualTCPposition);
  ros::Subscriber done = n.subscribe("/es_master/done", 10, doneCallback);
  sterowanie = n.advertise<geometry_msgs::PoseStamped>("sterowanie",1000);


  const double maxTimeWithoutHex = 0.2;
  const double maxTimeWithoutActualPosition = 0.2;
  const double maxTimeWithoutAcceptation = 0.5;

  lastActualPoseTime = ros::Time::now();
  lastHexTime = ros::Time::now();
  
/////////////////////////////////////////////////////Do zakomentowania
  //bool kierunek = true;
////////////////////////////////////////////////////

  bool nextPoseExist = false;
  bool messegeExist = false;
  resetForces();
  ros::Time now;

  ros::Rate loop_rate(loopRate); 
  
  while (ros::ok())
  {  
    now = ros::Time::now();
    /////////////////////////////////////////// symulowanie hexa
    // if(kierunek)
    //   actualForces.force.z += 1.0/loopRate;
    // else
    //   actualForces.force.z -= 1.0/loopRate;

    // if (actualForces.force.z > 5.0)
    //   kierunek =false;
    // else if (actualForces.force.z < -5.0)
    //   kierunek = true;

    // if(czyJestSila(actualForces))
    //   anyForce = true;
    /////////////////////////////////////////

    // Sprawdzam czy otrzymuje dene z czujnika
    if((now - lastHexTime) >= ros::Duration(maxTimeWithoutHex))
    {
      ROS_WARN("No connection with HEX!");
      resetForces();
    }
    
    // Sprawdzam czy otrzymuje dane o aktualnej pozycji punktu TCP robota
    if((now - lastActualPoseTime) >= ros::Duration(maxTimeWithoutActualPosition))
    {
      ROS_WARN("No connection with robot!");
      iHaveActualPose = false;
    }      

    // Jesli odebrano chociaz raz informacje o aktualnej pozycji to licze nowa pozycje 
    if(iHaveActualPose)
    {    
      findTransformToNextPosition(actualForces);
      if(calculateGlobalTargetPosition())
        nextPoseExist = true;  
    }

    // Jesli nowa pozycja istnieje, poprzedni ruch zostal wykonany, oraz czujnik wykrywa jakakolwiek sile to przygotowuje nowa wiadomosc
    if(nextPoseExist && moveIsDone && anyForce)
    { 
      messege = prepareMessege(nextPose);
      messegeExist=true;
      moveIsDone = false;
      nextPoseAccepted = false; 
    }

    // Jesli przez 0.5s od wygenerowania wiadomosci nie dostano informacji o akceptacji nowej pozycji przez robota to wyswietla blad oraz przerywa dzialanie programu
    if(!nextPoseAccepted)
    {
      if((now  - messege.header.stamp) > ros::Duration(maxTimeWithoutAcceptation))
      {
        ROS_ERROR("Next pose not accepted!");
        //return(0);
      }     
    }

    // Jesli przygotowano jakakolwiek wiadomosc to ja publikuj
    if(messegeExist)
      sterowanie.publish(messege);
    
    // zabezpieczenie aby korzystac zawsze z aktualnych danych z czujnika
    anyForce = false;

    // Uruchomienie funkcji obslugi odbierania danych oraz oczekiwanie w celu zachowania stalej czestatliwosci
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}

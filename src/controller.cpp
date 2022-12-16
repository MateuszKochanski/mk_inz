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
const int loopRate = 100;
bool iHaveActualPose = false;
bool moveIsDone = true;
bool anyForce = false;
double timeWithoutDone;
bool nextPoseAccepted = true;
ros::Time lastActualPoseTime;
ros::Time lastHexTime;

//sprawdza czy wartosc ktorejs z sil lub momentu przekracza pewien prog
bool nonzeroForce(geometry_msgs::Wrench hex)
{
  if(abs(hex.force.x) > 0.5 || abs(hex.force.y) > 0.5 || abs(hex.force.z) > 0.5 || abs(hex.torque.z) > 0.1 )
    return true;
  else
    return false;
}

//Pozbywanie sie zaklocen przy malych wartosciach sil
double cutAndScale(double wartosc, double prog, double skala)
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

//Obliczenia translacji i rotacji z aktualnej pozycji do nowego punktu
void findTransformToNextPosition(geometry_msgs::Wrench forces)
{

  static tf::TransformBroadcaster br;
  double progF = 0.5;
  double progT = 0.1;
  double scale = 0.01;
  double aScale = 1.5;
  double x, y, z, rx, ry, rz;

  tf::Quaternion angleTransform;
  
  x = cutAndScale(forces.force.x, progF, scale);
  y = cutAndScale(forces.force.y, progF, scale);
  z = cutAndScale(forces.force.z, 0.7, scale);

  rx = cutAndScale(forces.torque.x, progT, 0);
  ry = cutAndScale(forces.torque.y, progT, 0);
  rz = cutAndScale(forces.torque.z, progT, aScale);

  fprintf(stdout,"Fx: %.2f N Fy: %.2f N Fz: %.2f N Tz: %.2f dx: %.4f m dy: %.4f m dz: %.4f drz: %.4f rad\r\n",
    actualForces.force.x, actualForces.force.y, actualForces.force.z, actualForces.torque.z, x, y, z, rz);

  angleTransform.setRPY(rx, ry, rz);

  tf::Transform transform(angleTransform, tf::Vector3(-y, x, z));

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "TCP", "tmp"));
}

//Obliczanie polorzenia i orientacji nastepnego punktu
bool calculateNextPositionInGlobalFrame()
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

    tf::Matrix3x3 m(transform.getRotation());
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    yaw = yaw*(180.0/3.14);
    pitch = pitch*(180.0/3.14);
    roll = roll*(180.0/3.14);

    nextPose.orientation.x = yaw; 
    nextPose.orientation.y = pitch;
    nextPose.orientation.z = roll;

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

//wyzerowanie aktualnych wartosci sil i momentow 
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

//Zamiana kwaterionow z geometry_msgs na typ tf
tf::Quaternion msgToTfDatatype(geometry_msgs::Quaternion msg)
{
  tf::Quaternion a;
  a.setX(msg.x);
  a.setY(msg.y);
  a.setZ(msg.z);
  a.setW(msg.w);
  return a;
}

//odebranie wiadomosci z czujnika sily i momentu
void sensorCallback(const geometry_msgs::Wrench& msg)
{
  if(nonzeroForce(msg))
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

//odebranie wiadomosci potwierdzenia odebrania danych oraz wykonania ruchu
void feedbackCallback(const std_msgs::Header head)
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

//Odebranie wiadomosci o aktualnej pozycji
void actualPositionCallback(const geometry_msgs::Pose& msg)
{
  static tf::TransformBroadcaster br;
  tf::Quaternion a = msgToTfDatatype(msg.orientation);
  
  tf::Transform transform(a, tf::Vector3(msg.position.x, msg.position.y, msg.position.z));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "TCP"));

  lastActualPoseTime = ros::Time::now();
  iHaveActualPose = true;
}

//Przygotowanie wiadomosci zawierajacej nowa pozycje
geometry_msgs::PoseStamped prepareMessege(geometry_msgs::Pose pose)
{
  geometry_msgs::PoseStamped msg;
  msg.pose = pose;

  msg.header.frame_id = "NextPose";
  msg.header.stamp = ros::Time::now();

  return msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "forceController");
  ros::NodeHandle n;

  ros::Subscriber hex = n.subscribe("hex", 10, sensorCallback);
  ros::Subscriber TCP = n.subscribe("/es_arm/cartesian_pose", 10, actualPositionCallback);
  ros::Subscriber done = n.subscribe("/es_master/done", 10, feedbackCallback);
  sterowanie = n.advertise<geometry_msgs::PoseStamped>("sterowanie",1000);

  const double maxTimeWithoutHex = 0.2;
  const double maxTimeWithoutActualPosition = 0.2;
  const double maxTimeWithoutAcceptation = 0.5;

  lastActualPoseTime = ros::Time::now();
  lastHexTime = ros::Time::now();

  bool nextPoseExist = false;
  bool messegeExist = false;
  resetForces();
  ros::Time now;

  ros::Rate loop_rate(loopRate); 
  
  while (ros::ok())
  {  
    now = ros::Time::now();
  
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
      if(calculateNextPositionInGlobalFrame())
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

/*
This is a C++ script that subscribes to detection data, being published by
the sensorPub node, and relays motor commands to the bodyMotors and
neckMotors nodes. Behaviors defined in the main() function are tweaked as
desired.

Of note is the fact that the conductor node uses a class method 
(DetectionListener::getDetectionResult()) to retreive information on the 
type of joint detected on callback. See the ROS page on Class Methods as 
Callbacks for more details.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pipebot/sensorData.h"
#include "pipebot/sensorDetection.h"
#include "pipebot/bodyMotorsSrv.h"
#include "pipebot/neckMotorsSrv.h"

// custom class that allows detectionCallback to access detection results
class DetectionListener
{
public:
  void detectionCallback(const pipebot::sensorDetection::ConstPtr& msg); 
  int getDetectionResult();

private:
  int detectionResult;

};

int DetectionListener::getDetectionResult()
{
  return detectionResult;
}

// callback function for the detection subscriber; called when a detection
// result is published
void DetectionListener::detectionCallback(const pipebot::sensorDetection::ConstPtr& msg)
{
  // alter detectionResult to indicate
  // pertinent sensor information (joint detection)
  detectionResult = msg->data;
}

void setNeckMotor(int motorSignal, pipebot::neckMotorsSrv service, ros::ServiceClient client)
{
  service.request.signal = motorSignal;
  if (client.call(service))
  { 
    ROS_INFO("Neck Motor service call successful.");
  }
  else
  {
     ROS_ERROR("Failed to call service for Neck Motors");
  }
}

void setBodyMotors(int motorSignal, pipebot::bodyMotorsSrv service, ros::ServiceClient client)
{
  service.request.signal = motorSignal;
  if (client.call(service))
  { 
    ROS_INFO("DC Motor service call successful.");
  }
  else
  {
     ROS_ERROR("Failed to call service for DC Motors");
  }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "conductor");
  
  ros::NodeHandle n;
  
  // subscriber to detection results
  DetectionListener detectionListener;
  ros::Subscriber sub = n.subscribe("detection", 10, &DetectionListener::detectionCallback, &detectionListener);
  
  // client for body motors
  ros::ServiceClient bodyMotorClient = n.serviceClient<pipebot::bodyMotorsSrv>("bodyMotorSignal");
  pipebot::bodyMotorsSrv srvDCMotor;
  
  // client for neck motors
  ros::ServiceClient neckMotorClient = n.serviceClient<pipebot::neckMotorsSrv>("neckMotorSignal");
  pipebot::neckMotorsSrv srvNeckMotor;
  
  int dR;
  bool latched = false;
  int straight = 0;
  int right = 1;
  int left = 2;
  ros::Rate loop_rate(10);
  
  ROS_INFO("Conductor initialized.");
  
  while (ros::ok())
  {
    // check the value of detectionResult
    dR = detectionListener.getDetectionResult();

    // if sensor(s) do NOT detect a close object, motors set to 35% and 
    // neck is placed in neutral position
    if ((dR == 0) and (latched))
    {
      // attempt to set motors to 35%
      setBodyMotors(1,srvDCMotor,bodyMotorClient);
      latched = false;

      // attempt to place neck in neutral
      setNeckMotor(straight,srvNeckMotor,neckMotorClient);
    }


    // DEMO SPECIFIC: behavior designed specifically for demo-ing YRA joint
    if ((dR == 4) and (not latched))
    {
      // attempt to stop body motors
      setBodyMotors(0,srvDCMotor,bodyMotorClient);
      latched = true;
      
      //ros::Duration(5.0).sleep(); // wait for 5 seconds

      // YRA will always require a right-hand turn
      setNeckMotor(right,srvNeckMotor,neckMotorClient);
      
      // attempt to speed up motors to 50%
      setBodyMotors(2,srvDCMotor,bodyMotorClient);
      
      ros::Duration(6.25).sleep(); // wait for 5 seconds
      
      // attempt to re-straighten neck
      setNeckMotor(straight,srvNeckMotor,neckMotorClient);
      
    }

    /*
    // if sensor(s) detect a close object, stop motors and turn neck
    if(((dR == 1) or (dR == 2) or (dR == 4)) and (not latched))
    {
      //// attempt to stop body motors ////
      setBodyMotors(0,srvDCMotor,bodyMotorClient);
      latched = true;
      
      ros::Duration(5.0).sleep(); // wait for 5 seconds (for demo)

      //// attempt to turn neck ////
      if ((dR == 1) or (dR == 4))    // right-hand joint
      {
        setNeckMotor(right,srvNeckMotor,neckMotorClient);
      }

      if (dR == 2)    // TLS joint
      {
        setNeckMotor(left,srvNeckMotor,neckMotorClient);
      }
      
       //// attempt to speed up motors ////
      setBodyMotors(2,srvDCMotor,bodyMotorClient);
      
      //ros::Duration(5.0).sleep(); // wait for 5 seconds
      
      //// attempt to re-straighten neck ////
      //setNeckMotor(straight,srvNeckMotor,neckMotorClient);
      
    }*/
    
    ///////////////////////////////////////////////////////////////////////////
    //// U - JOINT ////
    if (((dR == 10) or (dR == 11)) and (not latched))
    {
      // attempt to turn neck
      if (dR == 10)  // right-hand U-Joint
      {
        setNeckMotor(right,srvNeckMotor,neckMotorClient);
      }

      if (dR == 11)    // left-hand U-Joint
      {
        setNeckMotor(left,srvNeckMotor,neckMotorClient);
      }
      
      // attempt to speed up body motors
      setBodyMotors(2,srvDCMotor,bodyMotorClient);
      
      latched = true;

    }


    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}

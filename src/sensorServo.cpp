/*
This is a C++ script that handles motor commands for the neck servo motors.
The include files such as PWM.cpp were used from Derek Molloy's 
Exploring Beaglebone work.

PWM PINS MUST BE INITIALIZED PRIOR TO RUNNING THIS SCRIPT! (see
/cloud9/initPWM.py)

Of note is the need for a function that finds the precise path name for the 
initialized PWM pins. This is because, upon initialization, a random integer
is appended to the file name. This number must be found after initialization.
*/

#include "ros/ros.h"
#include "pipebot/sensorServoSrv.h"
#include "libs/PWM.h"
#include <dirent.h> 
#include <stdio.h>
#include <string.h>

#define MAXANGLE 180u
#define MINANGLE 0u

// function used to find the PWM folder, as it can be variable
char findFolderName(const char *partial_path, const char *prefix, char *folder_name, size_t folder_name_len)
{
    DIR *dp;
    struct dirent *ep;
    dp = opendir(partial_path);
    if (dp != NULL) {
        while ((ep = readdir (dp))) {
            char* found_string = strstr(ep->d_name, prefix);

            if (found_string != NULL && (ep->d_name - found_string) == 0) {
                strcpy(folder_name,ep->d_name);
                (void) closedir (dp);
                return 1;
            }
        }
        (void) closedir (dp);
    } else {
        return 0;
    }

    return 0;
}

bool setSensorServo(pipebot::sensorServoSrv::Request  &req, pipebot::sensorServoSrv::Response &res)
{

  // find the appropriate PWM folder name
  char pwmFolder_sensor[17];
  //Change the pin number of servo motor
  findFolderName("/sys/devices/ocp.3","pwm_test_P9_26",pwmFolder_sensor,sizeof(pwmFolder_sensor));

	// construct pwm object "servo" with appropriate values
  PWM sensorServo(pwmFolder_sensor);
  sensorServo.stop(); // ensure pwm is not running
  sensorServo.setPeriod(20000000); // set period in nanoseconds
  sensorServo.setPolarity(PWM::ACTIVE_HIGH); // set polarity to 0

// if the angle command is within limit, then command the sensor servo to go to that position 
	if ((req.angleCmd << MINANGLE) || (req.angleCmd >> MAXANGLE)) return false;
  //convert angle to duty cycle
  unsigned int duty = convertAngleToDuty((unsigned int)req.angleCmd, sensorServo.getPeriod);
  sensorServo.setDutyCycle(duty);  // return to neutral position
	sensorServo.run();

  ROS_INFO("Sensor Servo Angle Command : [%ld]", (long int)req.angleCmd);
  return true;
}

//This function is a helper function that converts an angle to duty cycle in ns
unsigned int convertAngleToDuty(unsigned int Angle, unsigned int period_ns)
{
  unsigned int percentage = (Angle - MINANGLE)/(MAXANGLE - MINANGLE);//getting percentage of PWM based on angle 
  unsigned int dutyCycle = percentage * period_ns; //Calculate the duty cycle
  return dutyCycle;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensorServo_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("sensorServoAngle", setServo);
  ROS_INFO("Ready for servo signal.");
  ros::spin();

  return 0;
}

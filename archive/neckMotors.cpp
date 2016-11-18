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
#include "pipebot/neckMotorsSrv.h"
#include "libs/PWM.h"
#include <dirent.h> 
#include <stdio.h>
#include <string.h>

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

bool setServo(pipebot::neckMotorsSrv::Request  &req, pipebot::neckMotorsSrv::Response &res)
{

  // find the appropriate PWM folder name
  char pwmFolder_neck[17];
  findFolderName("/sys/devices/ocp.3","pwm_test_P9_22",pwmFolder_neck,sizeof(pwmFolder_neck));

	// construct pwm object "servo" with appropriate values
  PWM servo(pwmFolder_neck);
  servo.stop(); // ensure pwm is not running
  servo.setPeriod(20000000); // set period in nanoseconds
  servo.setPolarity(PWM::ACTIVE_HIGH); // set polarity to 0
  servo.setDutyCycle(1500000u);  // neutral position
  servo.run(); // put neck into neutral

  // the motor service request indicates desired motor behavior
	if (req.signal == 0)
	{
    servo.setDutyCycle(1500000u);  // return to neutral position
		servo.run();
	}

	if (req.signal == 1)
	{
    servo.setDutyCycle(2500000u); // turn RIGHT
		servo.run();
	}

  if (req.signal == 2)
  {
    servo.setDutyCycle(500000u); // turn LEFT
    servo.run();
  }


  ROS_INFO("Neck motor signal: [%ld]", (long int)req.signal);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "neckMotors_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("neckMotorSignal", setServo);
  ROS_INFO("Ready for servo signal.");
  ros::spin();

  return 0;
}

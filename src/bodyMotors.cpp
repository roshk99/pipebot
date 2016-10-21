/*
This is a C++ script that handles motor commands for the DC body motors.
The include files such as DCMotor.cpp were used from Derek Molloy's 
Exploring Beaglebone work.

PWM PINS MUST BE INITIALIZED PRIOR TO RUNNING THIS SCRIPT! (see
/cloud9/initPWM.py)

Of note is the need for a function that finds the precise path name for the 
initialized PWM pins. This is because, upon initialization, a random integer
is appended to the file name. This number must be found after initialization.
*/

#include "ros/ros.h"
#include "pipebot/bodyMotorsSrv.h"
#include "libs/DCMotor.h"
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

// body motor callback function
bool setMotor(pipebot::bodyMotorsSrv::Request  &req, pipebot::bodyMotorsSrv::Response &res)
{

    // find the appropriate PWM folder name; note that the pin is always P9_42 and P9_16
    // but the rest of the directory name is random and needs to be determined.
    char pwmFolder_body_1[17];
    char pwmFolder_body_2[17];
    findFolderName("/sys/devices/ocp.3","pwm_test_P9_42",pwmFolder_body_1,sizeof(pwmFolder_body_1));
    findFolderName("/sys/devices/ocp.3","pwm_test_P9_16",pwmFolder_body_2,sizeof(pwmFolder_body_2));
    
    // create dcmotor object and apply appropriate action based on the received signal
    DCMotor dcmotor_1(new PWM(pwmFolder_body_1), 116);
    DCMotor dcmotor_2(new PWM(pwmFolder_body_2), 117);
    dcmotor_1.setSpeedPercent(65.0f);   // for some reason, this is actually
    dcmotor_2.setSpeedPercent(65.0f);   // 35%; likely related to polarity??
    
    if (req.signal == 0)
    {
    	dcmotor_1.stop();
    	dcmotor_2.stop();
    }
    
    if (req.signal == 1)
    {
    	dcmotor_1.go();
    	dcmotor_2.go();
    }
    
    if (req.signal == 2)
    {
    	dcmotor_1.setSpeedPercent(50.0f);
        dcmotor_2.setSpeedPercent(50.0f);
        dcmotor_1.go();
    	dcmotor_2.go();
    }

    ROS_INFO("Body motor signal: [%ld]", (long int)req.signal);
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bodyMotors_server");
    ros::NodeHandle n;
    
    // find the appropriate PWM folder name; note that the pin is always P9_42/P9_16, but the rest needs to be determined.
    char pwmFolder_body_1[17];
    char pwmFolder_body_2[17];
    findFolderName("/sys/devices/ocp.3","pwm_test_P9_42",pwmFolder_body_1,sizeof(pwmFolder_body_1));
    findFolderName("/sys/devices/ocp.3","pwm_test_P9_16",pwmFolder_body_2,sizeof(pwmFolder_body_2));
    
    DCMotor dcmotor1(new PWM(pwmFolder_body_1), 116);
    DCMotor dcmotor2(new PWM(pwmFolder_body_2), 117);
    dcmotor1.setSpeedPercent(65.0f);
    dcmotor2.setSpeedPercent(65.0f);
    
    dcmotor1.go();
    dcmotor2.go();
    
    ros::ServiceServer service = n.advertiseService("bodyMotorSignal", setMotor);
    ROS_INFO("Ready for motor signal.");
    ros::spin();
    
    return 0;
}

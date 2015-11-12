This is a brief overview of the pipebot package, geared towards the team
inhereting Team Serpentelligence's (2015-2016) work on the Pipebot project.

Team Serpentelligence was a senior design group of 4 MECH majors, and as such
there were a considerable number of electrical and software-related learning
curves to overcome. This README assumes no prior knowledge of ROS and limited
experience with coding. In addition, no promises of optimal code are made ;) 
Feel free to change the structure of the code completely if deemed necessary!



The pipebot package is currently comprised of 5 nodes. See the ROS wiki for a
description of ROS architecture and jargon. The nodes are as follows:

    sensorPub.py
    bodyMotors.cpp
    neckMotors.cpp
    conductor.cpp
    accelPub.py
    
    
All nodes can be launched simultaneuously, effectively starting the Pipebot with
a single command, using the command:

    roslaunch pipebot pipebot.launch
    
    
    
    
    

********************************************************************************
NOTE: 

    After editing code, re-compiling your package with catkin_make (see ROS 
    wiki) on the BBB can be a lengthy process. Always make sure the current date
    on the BBB is updated when you attempt catkin_make. This can be done by 
    executing the commands:
    
        cd /var/lib/cloud9
        ./startUSBNetwork.py
        
    INTERNET CONNECTIVITY IS REQUIRED TO DO THIS! Always make sure to check the 
    date, to make sure the update was successful, using the command:
    
        date
        
    If the update failed, you may need to reset your internet connection with 
    the BBB.
    
********************************************************************************
    
    
    
    
    
    
sensorPub.py:

    This is a Python script that reads the pin volatages that correspond to the 
    5 IR sensors. The values are then placed into respective buffers and 
    averaged, to reduce noise in the signal and create a more reliable reading. 
    These readings are then sent through a series of threshold checks to
    determine whether the readings match a joint pattern. This detection is 
    then published in the ROS framework.
    
    NOTE: Python may or may not be necessary. Previous work suggests there is a
    bug with the Beaglebone when reading the analog input pins using C++, so 
    Python was used instead.
    
    
bodyMotors.cpp:

    This is a C++ script that handles motor commands for the DC body motors.
    The include files such as DCMotor.cpp were used from Derek Molloy's 
    Exploring Beaglebone work.
    
    PWM PINS MUST BE INITIALIZED PRIOR TO RUNNING THIS SCRIPT! (see
    /cloud9/initPWM.py)
    
    Of note is the need for a function that finds the precise path name for the 
    initialized PWM pins. This is because, upon initialization, a random integer
    is appended to the file name. This number must be found after initialization.
    
    
neckMotors.cpp:

    This is a C++ script that handles motor commands for the neck servo motors.
    The include files such as PWM.cpp were used from Derek Molloy's 
    Exploring Beaglebone work.
    
    PWM PINS MUST BE INITIALIZED PRIOR TO RUNNING THIS SCRIPT! (see
    /cloud9/initPWM.py)
    
    Of note is the need for a function that finds the precise path name for the 
    initialized PWM pins. This is because, upon initialization, a random integer
    is appended to the file name. This number must be found after initialization.
    
    
conductor.cpp:
    
    This is a C++ script that subscribes to detection data, being published by
    the sensorPub node, and relays motor commands to the bodyMotors and
    neckMotors nodes. Behaviors defined in the main() function are tweaked as
    desired.
    
    Of note is the fact that the conductor node uses a class method 
    (DetectionListener::getDetectionResult()) to retreive information on the 
    type of joint detected on callback. See the ROS page on Class Methods as 
    Callbacks for more details.
    
    
accelPub.py:

    This is a Python script that reads the pin voltages that correspond to the 
    two accelerometer outputs (x-axis, z-axis). The values are then placed into 
    respective buffers and averaged, to reduce noise in the signal and create a 
    more reliable reading. Both the raw data and buffered data are then 
    published in the ROS framework.
    
    NOTE: Python may or may not be necessary. Previous work suggests there is a
    bug with the Beaglebone when reading the analog input pins using C++, so 
    Python was used instead.
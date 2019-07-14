#ifndef ACTIONLIB_SAMPLE_SERVER_H_
#define ACTIONLIB_SAMPLE_SERVER_H_

//Including some useful stuff
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
//Custom predefined action messages
#include <actionlib_samples/actionlib_sampleAction.h>

//включаем ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>


class Actionlib_Sample_Client{
    private:
        ros::NodeHandle n;

    public:
        Actionlib_Sample_Client();

};

#endif
//Andrew Tarnoff for EECS 373

#ifndef SIN_COMMANDER_SERVER_H_
#define SIN_COMMANDER_SERVER_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <ros/ros.h> //ALWAYS need to include this
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>

//message types used in this example code;  include more message types, as needed
#include <problem_set_3/sinCommanderAction.h>

// define a class, including a constructor, member variables and member functions
class SinCommanderServer {
public:
    SinCommanderServer(ros::NodeHandle* nodehandle); //Constructor
    void publishZero(std_msgs::Float64 zero);
    bool isRunningCb;
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // node handle
    ros::Publisher  sin_vel_pub_;
    problem_set_3::sinCommanderResult result_; // put results here
    actionlib::SimpleActionServer<problem_set_3::sinCommanderAction> sin_as_; //action server

    void sinServerCallback(const actionlib::SimpleActionServer<problem_set_3::sinCommanderAction>::GoalConstPtr& goal); //Callback function for sin commander server
};

#endif

// Andrew Tarnoff for EECS 373
// Problem Set 6

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string>
using namespace std;


//Global Variables for logical camera callback
bool g_takeSnapshot;
osrf_gear::LogicalCameraImage g_currLogicalImage;

void logicalCamera2CB(const osrf_gear::LogicalCameraImage& newImage) {
        if(g_takeSnapshot) {
                g_currLogicalImage = newImage;
                g_takeSnapshot = false;
        }
}

bool setConveyorPower(double power, ros::ServiceClient conveyor_client) {
        osrf_gear::ConveyorBeltControl conveyor_srv;

        //Starting up the Conveyor Belt
        g_takeSnapshot = true; //Initializes subscriber global variables
        ros::spinOnce();
        conveyor_srv.request.power = power;
        conveyor_client.call(conveyor_srv);
        while(!conveyor_srv.response.success) {
                ROS_WARN("Failed to command the Conveyor");
                conveyor_client.call(conveyor_srv);
                ros::Duration(0.5).sleep();
        }
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "problem_set_6_node");
        ros::NodeHandle n;

        //Initializing the Start Competition Service
        ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        std_srvs::Trigger startup_srv;

        //Initializing the Conveyor Belt Service
        ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");

        //Initializing the Drone Service
        ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
        osrf_gear::DroneControl drone_srv;

        //Initializing the LogicalCamera2 Subscriber
        ros::Subscriber logicalCamera2_sub = n.subscribe("/ariac/logical_camera_2",1,logicalCamera2CB);

        //Starting up the competition
        startup_client.call(startup_srv);
        while(!startup_srv.response.success) {
                ROS_WARN("Failed to Start Competition");
                startup_client.call(startup_srv);
                ros::Duration(0.5).sleep();
        }
        ROS_INFO("Started the Competition");

        //Starting up the Conveyor Belt
        setConveyorPower(100.0, conveyor_client);
        ROS_INFO("Started the Conveyor");

        //Moving the conveyer belt until it detects a box
        while(g_currLogicalImage.models.size() == 0) {
                //Keep the Conveyor Moving until there is a model detected
                g_takeSnapshot = true;
                ros::spinOnce();

        }
        ROS_INFO("Detected a Box underneath LogicalCamera2");

        double tolerance = 0.02;
        //Moving the conveyer belt until it detects that the box is centered under the camera (with a tolerance of "tolerance")
        while(abs(g_currLogicalImage.models[0].pose.position.z) >= tolerance) {
                //Keep the Cnveyor Moving until there is a box within "tolerance" meters of the center of the camera
                g_takeSnapshot = true;
                ros::spinOnce();
        }

        //Stop the Conveyor Belt
        setConveyorPower(0.0, conveyor_client);
        ROS_INFO("Stopped the Box underneath LogicalCamera2 and DELAYING for 5 seconds");

        //Delay for 5 seconds
        ros::Duration(1.0).sleep();

        //Turn the conveyer back on so that the box moves to the pickup zone
        setConveyorPower(100.0, conveyor_client);
        ROS_INFO("Turned the Conveyor back on to move box to pickup zone");


        //Calling the Drone
        drone_srv.request.shipment_type = "dummy";
        drone_client.call(drone_srv);
        while(!drone_srv.response.success) {
                ROS_WARN("Failed to call the Drone");
                drone_client.call(drone_srv);
                ros::Duration(0.5).sleep();
        }
        ROS_INFO("Called the Drone");

        return 0;
}

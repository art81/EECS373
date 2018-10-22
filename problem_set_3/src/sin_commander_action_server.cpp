//Andrew Tarnoff for EECS 373

#include <ros/ros.h>
#include <problem_set_3/sin_commander_server.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv) {
        ros::init(argc, argv, "sin_commander_action_server"); // name this node
        ros::NodeHandle nh;

        SinCommanderServer sinCommanderActionServer(&nh);

        ROS_INFO("Waiting for Goal to be sent from Client (spin)");

        //Publishing zeros at 100Hz as long as the callback function isn't running
        std_msgs::Float64 zero;
        zero.data = 0.0;
        ros::Rate zeroNap(100);

        while(ros::ok()) {
                if(!sinCommanderActionServer.isRunningCb) {
                        sinCommanderActionServer.publishZero(zero);
                        ros::spinOnce();
                        zeroNap.sleep();
                }
        }

        return 0;
}

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel

double sample_dt = 0.001; //specify a sample period of 10ms
geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR

void moveForward(double time, ros::Publisher twist_commander, ros::Rate loop_timer) {
        double timer = 0.0;
        double speed = 1.0;

        twist_cmd.linear.x = speed; //command to move forward
        twist_cmd.angular.z = 0.0;
        while(timer < time) {
                twist_commander.publish(twist_cmd);
                timer+=sample_dt;
                loop_timer.sleep();
        }
}

void turn(double angle, double angularSpeed, ros::Publisher twist_commander, ros::Rate loop_timer) {
        double totalTime = angle/0.5; //How much time you must spin at 0.5rad/s to get desired angle

        twist_cmd.linear.x = 0.0; //stop moving forward
        twist_cmd.angular.z = angularSpeed; //and start spinning in place
        double timer = 0.0; //reset the timer
        while(timer < totalTime) {
                twist_commander.publish(twist_cmd);
                timer+=sample_dt;
                loop_timer.sleep();
        }
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "stdr_commander");
        ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
        ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
        //some "magic numbers"
        double quarterTurnRad = 1.57;

        // start with all zeros in the command message; should be the case by default, but just to be safe..
        twist_cmd.linear.x=0.0;
        twist_cmd.linear.y=0.0;
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;

        ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate
        double timer=0.0;
        //start sending some zero-velocity commands, just to warm up communications with STDR
        for (int i=0; i<10; i++) {
                twist_commander.publish(twist_cmd);
                loop_timer.sleep();
        }

        moveForward(3.5, twist_commander, loop_timer); //Move forward 3m

        turn(quarterTurnRad, 0.5, twist_commander, loop_timer); //Turn 90 degrees counter clockwise

        moveForward(2.5, twist_commander, loop_timer); //Move forward 3m

        turn(quarterTurnRad, -0.5, twist_commander, loop_timer); //Turn 90 degrees clockwise

        moveForward(4.0, twist_commander, loop_timer); //Move forward 4m

        turn(quarterTurnRad, 0.5, twist_commander, loop_timer); //Turn 90 degrees counter clockwise

        moveForward(3.0, twist_commander, loop_timer); //Move forward 3m

        turn(quarterTurnRad, 0.5, twist_commander, loop_timer); //Turn 90 degrees counter clockwise

        moveForward(2.0, twist_commander, loop_timer); //Move forward 1m

        turn(quarterTurnRad, -0.5, twist_commander, loop_timer); //Turn 90 degrees clockwise

        moveForward(5.5, twist_commander, loop_timer); //Move forward 5m

        turn(quarterTurnRad, 0.5, twist_commander, loop_timer); //Turn 90 degrees counter clockwise

        moveForward(4.0, twist_commander, loop_timer); //Move forward 4m

        turn(quarterTurnRad, -0.5, twist_commander, loop_timer); //Turn 90 degrees clockwise

        moveForward(1.0, twist_commander, loop_timer); //Move forward 1m

        turn(quarterTurnRad, 0.5, twist_commander, loop_timer); //Turn 90 degrees counter clockwise

        moveForward(1.75, twist_commander, loop_timer); //Move forward 2m

        //halt the motion
        twist_cmd.angular.z=0.0;
        twist_cmd.linear.x=0.0;
        for (int i=0; i<10; i++) {
                twist_commander.publish(twist_cmd);
                loop_timer.sleep();
        }
        //done commanding the robot; node runs to completion
}

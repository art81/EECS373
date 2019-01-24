// Andrew Tarnoff for EECS 373

#include <ros/ros.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;

int main(int argc, char** argv) {
        ros::init(argc, argv, "irb120_traj_sender");
        ros::NodeHandle nh;

        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
        trajectory_msgs::JointTrajectory traj;

        //Populate this message type and publish to complete assignment
        /**
           std_msgs/Header header
              time stamp //intiialize this right before sending!
           string[] joint_names
           trajectory_msgs/JointTrajectoryPoint[] points
              float64[] positions
           duration time_from_start
         **/

        //Intializing Joint Names Variable
        traj.points.clear();
	traj.joint_names.clear();
        traj.joint_names.push_back("joint1");
        traj.joint_names.push_back("joint2");
        traj.joint_names.push_back("joint3");
        traj.joint_names.push_back("joint4");
        traj.joint_names.push_back("joint5");
        traj.joint_names.push_back("joint6");

        //Variables for numPoints
        double dt = 0.01;              //each point in traj is 0.01 seconds apart
        double finalTime = 15;         //15 second trajectory
        int numPoints = finalTime/dt;  //Number of points in traj
        double PI = 4.0*atan(1);       //Value for Pi since atan(1) = 0.25*PI

        //Sin wave parameters for commanded joint angles
        double amp1 = 1;
        double amp2 = 1;
	double amp3 = 0.5;
        double freq1 = 0.75;
        double freq2 = 1.5;
	double freq3 = 2.25;
	double theta1 = PI;
	double theta2 = 0;
	double theta3 = PI/2;

        //For every point, clear the positions variable. Re define it and then add that onto the points list in traj
        double t = 0.0;
        trajectory_msgs::JointTrajectoryPoint newPoint;
	traj.points.clear();
        for(int i = 0; i<numPoints; i++) {
                //Fill in positions vector inside of new point
                newPoint.positions.clear();

                newPoint.positions.push_back(amp1*sin(2*PI*freq1*(t-theta1))); //Joint 1
                newPoint.positions.push_back(amp2*sin(2*PI*freq2*(t-theta2))); //Joint 2
                newPoint.positions.push_back(amp3*sin(2*PI*freq3*(t-theta3))); //Joint 3
                newPoint.positions.push_back(0.0); //Joint 4
                newPoint.positions.push_back(0.0); //Joint 5
                newPoint.positions.push_back(0.0); //Joint 6

		//Sends a trajectory only containing home first, then later sends actual trajectory
		if(i == 0) {
			traj.header.stamp = ros::Time::now();
			pub.publish(traj);
			ros::Duration(2.0).sleep();
		}

                //Fill in time_from_start in new point and increment next time
                newPoint.time_from_start = ros::Duration(t);
                t += dt;

                //Add new point to list of points in traj
                traj.points.push_back(newPoint);
        }

        traj.header.stamp = ros::Time::now();
        pub.publish(traj);

        //Waits 3 seconds before dying to make sure that trajectory was published
        ros::Rate publish_wait(1.0);
        for(int i = 0; i<3; i++) {
                publish_wait.sleep();
        }

        return 0;
}

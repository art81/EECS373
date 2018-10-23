//irb120_reactive_task_commander.cpp
//this version is a variation on irb120_task_commander, but adds an action client of
//the magic_object_finder to make the robot move in response to perceived objects

//Andrew Tarnoff for EECS 373: MEAT OF THE CODE STARTS AT LINE 232!

#include <ros/ros.h>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std; // avoids having to say: std::string, std::cout, etc
#include <irb120_fk_ik/irb120_kinematics.h>  //access to forward and inverse kinematics
#include <fk_ik_virtual/fk_ik_virtual.h> //defines the base class with virtual fncs
// this is useful to keep the motion planner generic
#include "robot_specific_fk_ik_mappings.h" //these two files are needed to provide robot-specific info to generic planner
#include "robot_specific_names.h"

#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

//add these to use the "magic" object finder action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>

//the following will be useful when need tool transforms
//#include <tf/transform_listener.h>
//#include <xform_utils/xform_utils.h>

//XformUtils xformUtils; //handy conversion utilities--but don't need these yet

//some magic numbers--place at the top of the program
std::vector<double> g_planner_joint_weights{3, 3, 2, 1, 1, 0.5}; //specify weights to use for planner optimization

//another magic value: hard-coded name of object of interest
string g_object_name("gear_part");  //hard-coded object name; edit this for different objects
int g_found_object_code; //global to communicate between callback and main: true if named object was found
geometry_msgs::PoseStamped g_perceived_object_pose; //global to communicate between callback and main: pose  of found object

ros::Publisher *g_pose_publisher; //make this global so callback can access it--for displaying object frames in rviz

CartTrajPlanner *pCartTrajPlanner; //does  not  have to be global, unless needed by other functions

//arm pose in joint space; the only reason this is global is that it will be useful, in the future, for it
//to be updated by a subscriber to joint_states
Eigen::VectorXd g_q_vec_arm_Xd;

//this callback function receives a result from the magic object finder action server
//it sets g_found_object_code to true or false, depending on whether the  object was found
//if the object was found, then components of g_perceived_object_pose are filled in
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
                        const magic_object_finder::magicObjectFinderResultConstPtr& result) {
        ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
        g_found_object_code=result->found_object_code;
        ROS_INFO("got object code response = %d; ",g_found_object_code);
        if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND) {
                ROS_WARN("object-finder responded: object not found");
        }
        else if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
                ROS_INFO("found object!");
                g_perceived_object_pose= result->object_pose;
                ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                         g_perceived_object_pose.pose.position.y,
                         g_perceived_object_pose.pose.position.z);

                ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                         g_perceived_object_pose.pose.orientation.y,
                         g_perceived_object_pose.pose.orientation.z,
                         g_perceived_object_pose.pose.orientation.w);
                g_pose_publisher->publish(g_perceived_object_pose); //this is to enable display of pose of found object in rviz
        }
        else {
                ROS_WARN("object not found!");
        }
}

//The following makes an inquiry for the pose of the part of interest and stores it in g_perceived_object_pose
void findGearObjectPose(actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> &object_finder_ac) {
        //specify the part name, send it in the goal message, wait for and interpret the result
        magic_object_finder::magicObjectFinderGoal object_finder_goal; //instantiate goal message to communicate with magic_object_finder
        object_finder_goal.object_name = g_object_name.c_str(); //convert string object to old C-style string data
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server

        bool finished_before_timeout = false;
        finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
        //NOTE: could do something else here (if useful) while waiting for response from action server
        if (!finished_before_timeout) {
                ROS_ERROR("giving up waiting on result "); //this should not happen; should get result of found or not-found
                system("rosnode kill task_commander_with_coords"); //Kills this node from this method instead of from main
        }
        //check the result code to see if object was found or not
        if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND)   {
                ROS_INFO("found object!");
        } else {
                ROS_ERROR("object not found!  Quitting");
                system("rosnode kill task_commander_with_coords"); //Kills this node from this method instead of from main
        }
        //Part pose is now in g_perceived_object_pose.  Use it to compute robot motion
}

std::vector<Eigen::VectorXd> moveFromAToB(Eigen::VectorXd A, Eigen::Affine3d B, int nsteps, double arrival_time, ros::Publisher traj_publisher) {
        trajectory_msgs::JointTrajectory new_trajectory; //will package trajectory messages here

        std::vector<Eigen::VectorXd> optimal_path;
        optimal_path.clear();

        if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(A, B, nsteps, optimal_path)) {
                ROS_ERROR("no feasible IK path for specified Cartesian motion; quitting");
                system("rosnode kill task_commander_with_coords");
        }

        pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
        traj_publisher.publish(new_trajectory); //publish the trajectory
        ros::Duration(arrival_time).sleep(); //wait for the motion

        ROS_INFO("done with last trajectory");

        return optimal_path;
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "task_commander_with_coords"); // name this node
        ros::NodeHandle nh; //standard ros node handle
        Eigen::Affine3d start_flange_affine, goal_flange_affine; //specify start and goal in Cartesian coords
        std::vector<Eigen::VectorXd> optimal_path; //a path in joint space is a sequence of 6-DOF joint-angle specifications
        trajectory_msgs::JointTrajectory new_trajectory; //will package trajectory messages here

        //set up an action client to query object poses using the magic object finder
        actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> object_finder_ac("object_finder_action_service", true);
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = false;
        while ((!server_exists)&&(ros::ok())) {
                server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); //
                ros::spinOnce();
                ros::Duration(0.5).sleep();
                ROS_INFO("retrying...");
        }
        ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server;
        ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
        g_pose_publisher = &pose_publisher;

        //the following is an std::vector of affines.  It describes a path in Cartesian coords, including orientations
        //not needed yet; is constructed inside the generic planner by interpolation
        //std::vector<Eigen::Affine3d> affine_path;
        Eigen::Matrix3d R_down; //define an orientation corresponding to toolflange pointing down
        Eigen::Vector3d x_axis, y_axis, z_axis, flange_origin;
        z_axis << 0, 0, -1; //points flange down
        x_axis << -1, 0, 0; //arbitrary
        y_axis = z_axis.cross(x_axis); //construct y-axis consistent with right-hand coordinate frame
        R_down.col(0) = x_axis;
        R_down.col(1) = y_axis;
        R_down.col(2) = z_axis;
        flange_origin << 0.2, 0, 0.01; //SHOULD GET FIXED: hard-coded pose can result in ugly/dangerous motion
        int nsteps = 5; //will need to specify how many interpolation points in Cartesian path; this is pretty coarse
        double arrival_time = 5.0; //will  need to specify arrival time for a Cartesian path

        //for this next line, I apparently did something wrong.  I should not have to  instantiate a cartesianInterpolator,
        //since the generic planner instantiates one.  But I get a compiler error.  Hmm...  Workaround.
        CartesianInterpolator cartesianInterpolator;

        g_q_vec_arm_Xd.resize(NJNTS); //generic vector resized to actual robot number of joints
        g_q_vec_arm_Xd << 0, 0, 0, 0, 0, 0; //assumes arm starts in this pose; better would be  to subscribe to joint_states to get actual angles

        //our irb120 control  interface uses this topic to receive trajectories
        ros::Publisher traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

        //somewhat odd construction: a pointer to an object of type CartTrajPlanner, with arguments provided
        //that are pointers to forward and inverse kinematic functions.  This is to keep the planner generic,
        //and defer WHICH robot FK and IK to use until run time; Uses virtual functions for this.
        pCartTrajPlanner = new CartTrajPlanner(pIKSolver, pFwdSolver, njnts);
        //the planner needs to define penalty weights to optimize a path
        pCartTrajPlanner->set_jspace_planner_weights(g_planner_joint_weights);
        //to fill out a trajectory, need to provide the joint names; these are contained in a robot-specific header file
        pCartTrajPlanner->set_joint_names(g_jnt_names);


        optimal_path.clear(); //reset this std::vector before  each use, else  will have old values persisting
        optimal_path.push_back(g_q_vec_arm_Xd); //start from current pose
        optimal_path.push_back(g_q_vec_arm_Xd); // go from current pose to current pose--not very useful; but can "warm up" control
        //publish/subscribe interface
        arrival_time = 1; //move should require zero time, but provide something small

        //function call from library (Class) CartTrajPlanner: converts a joint-space path to a joint-space trajectory
        pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);

        traj_publisher.publish(new_trajectory); //publish the trajectory;
        ros::Duration(1).sleep();

        //Andrew Tarnoff for EECS 373 MEAT OF THE CODE I WROTE AFTER THIS LINE

        //Getting user input of where the gear should be moved to
        double xDes, yDes;
        cout << "Enter an X-coord for the gear to be moved to: ";
        cin>>xDes;
        cout << "Enter a Y-coord for the gear to be moved to: ";
        cin>>yDes;

        //Finds the gear object pose and then continues if it found and breaks if not.
        //This code stores the pose in g_perceived_object_pose
        findGearObjectPose(object_finder_ac);

        //Parameter to figure out where to move the flange to so that the gear is in a desired position.
        double flangeRadius = 0.06;

        //Errors between where the gear is and where we need it to be
        double xError = xDes - g_perceived_object_pose.pose.position.x;
        double yError = yDes - g_perceived_object_pose.pose.position.y;
        Eigen::VectorXd initialVectorXd;
        Eigen::Affine3d goalAffine;

        //Dealing with the X-COORD of the gear!
        //This loop will continue until the error is low enough.
        //First pushes the gear where it should be and then moves the arm straight up and then
        //checks the error and continues if it needs to.
        int nSteps = 100;
        arrival_time = 3.0;
        double pushZ = 0.005; //Z Coord of the flange when pushing the gear
        double upZ = 0.15; //Z coord of the flange to rise to after pushing the gear

        while (abs(xError) >= 0.002 || abs(yError) >= 0.002) {

                double xMove = 0.0; //Variable to be incremented every loop so that the gear gets closer and closer to desired position
                while (abs(xError) >= 0.002) {
                        //XXXXXXXXX MOVES FLANGE NEXT TO GEAR TO BE PUSHED
                        //Initial position is where the last optimal path commanded. Now clear optimal path
                        //To be filled with new path
                        initialVectorXd = optimal_path.back();
                        optimal_path.clear();

                        //Set goal affine to where the gear is but 0.08m away so it can push. Either side of the gear depending which way it needs to be pushed
                        goalAffine.linear() = R_down; //Facing downwards
                        if (xError > 0) {
                                flange_origin << g_perceived_object_pose.pose.position.x - 0.08, g_perceived_object_pose.pose.position.y, pushZ;
                        } else {
                                flange_origin << g_perceived_object_pose.pose.position.x + 0.08, g_perceived_object_pose.pose.position.y, pushZ;
                        }
                        goalAffine.translation() = flange_origin;

                        optimal_path = moveFromAToB(initialVectorXd,goalAffine, nSteps, arrival_time, traj_publisher);
                        //XXXXXXXXX

                        //XXXXXXXXX MOVES FLANGE TO PUSH GEAR TO "DESIRED" POSITION
                        //Initial position is where the last optimal path commanded. Now clear optimal path
                        //To be filled with new path
                        initialVectorXd = optimal_path.back();
                        optimal_path.clear();

                        //Set goal affine to where the gear is desired to be but minus flangeRadius to account for size of gear
                        goalAffine.linear() = R_down; //Facing downwards
                        //Decides whether or not to end less than or greater then the actual desired position
                        //Based on which side the flange is on the gear
                        if(xError > 0) {
                                flange_origin << xDes - (flangeRadius - xMove), g_perceived_object_pose.pose.position.y, pushZ;
                        } else {
                                flange_origin << xDes + (flangeRadius - xMove), g_perceived_object_pose.pose.position.y, pushZ;
                        }

                        goalAffine.translation() = flange_origin;

                        optimal_path = moveFromAToB(initialVectorXd,goalAffine, nSteps, arrival_time, traj_publisher);
                        //XXXXXXXXX

                        //XXXXXXXXX MOVES FLANGE DIRECTLY UP WAITING FOR NEXT COMMAND TO MOVE
                        //Initial position is where the last optimal path commanded. Now clear optimal path
                        //To be filled with new path
                        initialVectorXd = optimal_path.back();
                        optimal_path.clear();

                        //Set goal affine to where the flange was but up to z = 0.2
                        goalAffine.linear() = R_down; //Facing downwards
                        if(xError > 0) {
                                flange_origin << xDes - (flangeRadius - xMove), g_perceived_object_pose.pose.position.y, upZ;
                        } else {
                                flange_origin << xDes + (flangeRadius - xMove), g_perceived_object_pose.pose.position.y, upZ;
                        }
                        goalAffine.translation() = flange_origin;

                        optimal_path = moveFromAToB(initialVectorXd,goalAffine, nSteps, arrival_time, traj_publisher);
                        //XXXXXXXXX

                        //Calculates new gear errors
                        findGearObjectPose(object_finder_ac);
                        xError = xDes - g_perceived_object_pose.pose.position.x;
                        yError = yDes - g_perceived_object_pose.pose.position.y;
                        xMove += 0.002;
                }


                //Dealing with the Y-COORD of the gear!
                //This loop will continue until the error is low enough.
                //First pushes the gear where it should be and then moves the arm straight up and then
                //checks the error and continues if it needs to.
                double yMove = 0.0; //Variable to be incremented every loop so that the gear gets closer and closer to desired position
                while (abs(yError) >= 0.002) {
                        //XXXXXXXXX MOVES FLANGE NEXT TO GEAR TO BE PUSHED
                        //Initial position is where the last optimal path commanded. Now clear optimal path
                        //To be filled with new path
                        initialVectorXd = optimal_path.back();
                        optimal_path.clear();

                        //Set goal affine to where the gear is but 0.08m away so it can push
                        goalAffine.linear() = R_down; //Facing downwards
                        if (yError > 0) {
                                flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y - 0.08, pushZ;
                        } else {
                                flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y + 0.08, pushZ;
                        }
                        goalAffine.translation() = flange_origin;

                        optimal_path = moveFromAToB(initialVectorXd,goalAffine, nSteps, arrival_time, traj_publisher);
                        //XXXXXXXXX

                        //XXXXXXXXX MOVES FLANGE TO PUSH GEAR TO "DESIRED" POSITION
                        //Initial position is where the last optimal path commanded. Now clear optimal path
                        //To be filled with new path
                        initialVectorXd = optimal_path.back();
                        optimal_path.clear();

                        //Set goal affine to where the gear is desired to be but minus flangeRadius to account for size of gear
                        goalAffine.linear() = R_down; //Facing downwards
                        //Decides whether or not to end less than or greater then the actual desired position
                        //Based on which side the flange is on the gear
                        if(yError > 0) {
                                flange_origin << g_perceived_object_pose.pose.position.x, yDes - (flangeRadius - yMove), pushZ;
                        } else {
                                flange_origin << g_perceived_object_pose.pose.position.x, yDes + (flangeRadius - yMove), pushZ;
                        }

                        goalAffine.translation() = flange_origin;

                        optimal_path = moveFromAToB(initialVectorXd,goalAffine, nSteps, arrival_time, traj_publisher);
                        //XXXXXXXXX

                        //XXXXXXXXX MOVES FLANGE DIRECTLY UP WAITING FOR NEXT COMMAND TO MOVE
                        //Initial position is where the last optimal path commanded. Now clear optimal path
                        //To be filled with new path
                        initialVectorXd = optimal_path.back();
                        optimal_path.clear();

                        //Set goal affine to where the flange was but up to z = 0.2
                        goalAffine.linear() = R_down; //Facing downwards
                        if(yError > 0) {
                                flange_origin << g_perceived_object_pose.pose.position.x, yDes - (flangeRadius - yMove), upZ;
                        } else {
                                flange_origin << g_perceived_object_pose.pose.position.x, yDes + (flangeRadius - yMove), upZ;
                        }
                        goalAffine.translation() = flange_origin;

                        optimal_path = moveFromAToB(initialVectorXd,goalAffine, nSteps, arrival_time, traj_publisher);
                        //XXXXXXXXX

                        //Calculates new gear errors
                        findGearObjectPose(object_finder_ac);
                        yError = yDes - g_perceived_object_pose.pose.position.y;
                        xError = xDes - g_perceived_object_pose.pose.position.x;
                        yMove += 0.002;
                }

                //Calculates new gear errors
                findGearObjectPose(object_finder_ac);
                yError = yDes - g_perceived_object_pose.pose.position.y;
                xError = xDes - g_perceived_object_pose.pose.position.x;
        }

        //Commands the arm to go back to home after the full motion
        g_q_vec_arm_Xd << 0, 0, 0, 0, 0, 0;
        optimal_path.clear(); //Clear optimal path.
        optimal_path.push_back(g_q_vec_arm_Xd); //start from current pose
        optimal_path.push_back(g_q_vec_arm_Xd); // go from current pose to current pose

        //Converts a joint-space path to a joint-space trajectory
        pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
        traj_publisher.publish(new_trajectory); //publish the trajectory;
}

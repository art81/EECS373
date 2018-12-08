//unload_box.cpp:
// moves a box under camera, then removes sensed parts from box

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include <inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include <box_inspector/box_inspector.h>

//conveyor interface communicates with the conveyor action server
#include <conveyor_as/ConveyorInterface.h>

//For starting the competition
#include <std_srvs/Trigger.h>


const double COMPETITION_TIMEOUT=500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
        part.name = model.type;
        part.pose.pose = model.pose;
        part.location = location; //by default
}

//Converts a quaternion to a 3 by 3 rotation matrix
Eigen::MatrixXd quaternionToMatrix(geometry_msgs::Quaternion quat) {
        Eigen::MatrixXd m(3,3);
        double x = quat.x;
        double y = quat.y;
        double z = quat.z;
        double w = quat.w;
        double x2 = x*x;
        double y2 = y*y;
        double z2 = z*z;
        double w2 = w*w;

        m.row(0) << 1-(2*y2)-(2*z2), 2*x*y-2*z*w, 2*x*z+2*y*w;
        m.row(1) << 2*x*y+2*z*w, 1-2*x2-2*z2, 2*y*z-2*x*w;
        m.row(2) << 2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x2-2*y2;

        return m;
}

//Converts a pose (containing x,y,z and quaternion) to a 4 by 4 transformation matrix
Eigen::MatrixXd poseToTransform(geometry_msgs::Pose pose) {
        Eigen::MatrixXd T(4,4);
        Eigen::MatrixXd R(3,3);

        R = quaternionToMatrix(pose.orientation);

        T.row(0) << R(0,0), R(0,1), R(0,2), pose.position.x;
        T.row(1) << R(1,0), R(1,1), R(1,2), pose.position.y;
        T.row(2) << R(2,0), R(2,1), R(2,2), pose.position.z;
        T.row(3) << 0,      0,      0,      1;

        return T;
}

//Finds which model in "models" is located closest to refPose
//This uses euclidean distance of x,y,z coordinates held in poses/transforms
osrf_gear::Model findClosestModel(Eigen::MatrixXd refTransform, vector<osrf_gear::Model> models) {
        double minDist = -1;
        int minIdx = 0;

        double refX = refTransform(0,3);
        double refY = refTransform(1,3);
        double refZ = refTransform(2,3);

        //Loops through every model, calculates euclidean distance to reference transform
        //Then sees if that is a new minimum and saves that minimum and the index where it occured
        for(int i = 0; i < models.size(); i++) {
                double currX = models[i].pose.position.x;
                double currY = models[i].pose.position.y;
                double currZ = models[i].pose.position.z;

                //Euclidean Distance
                double currDist = sqrt(pow(refX - currX,2) + pow(refY - currY,2) + pow(refZ - currZ,2));

                if(currDist < minDist) {
                        minDist = currDist;
                        minIdx = i;
                }
        }

        //Return the model that created the minimum distance to the reference transform
        return models[minIdx];
}

int main(int argc, char** argv) {
        // ROS set-ups:
        ros::init(argc, argv, "box_unloader"); //node name
        ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
        int ans;

        //Initializing the Start Competition Service
        ros::ServiceClient startup_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        std_srvs::Trigger startup_srv;

        ROS_INFO("instantiating a RobotBehaviorInterface");
        RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

        ROS_INFO("instantiating a ConveyorInterface");
        ConveyorInterface conveyorInterface(&nh);

        ROS_INFO("instantiating a BoxInspector");
        BoxInspector boxInspector(&nh);

        //instantiate an object of appropriate data type for our move-part commands
        inventory_msgs::Part current_part;

        geometry_msgs::PoseStamped box_pose_wrt_world; //camera sees box, coordinates are converted to world coords

        bool status;
        int nparts;

        //for box inspector, need to define multiple vectors for args,
        //box inspector will identify parts and convert their coords to world frame
        //in the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
        vector<osrf_gear::Model> desired_models_wrt_world;
        vector<osrf_gear::Model> satisfied_models_wrt_world;
        vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
        vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
        vector<osrf_gear::Model> missing_models_wrt_world;
        vector<osrf_gear::Model> orphan_models_wrt_world;
        vector<int> part_indices_missing;
        vector<int> part_indices_misplaced;
        vector<int> part_indices_precisely_placed;

        //Starting up the competition
        startup_client.call(startup_srv);
        while(!startup_srv.response.success) {
                ROS_WARN("Failed to Start Competition");
                startup_client.call(startup_srv);
                ros::Duration(0.5).sleep();
        }
        ROS_INFO("Started the Competition");

        //use conveyor action  server for multi-tasking
        ROS_INFO("getting a box into position: ");
        int nprint = 0;
        conveyorInterface.move_new_box_to_Q1(); //member function of conveyor interface to move a box to inspection station 1
        while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
                nprint++;
                if (nprint % 10 == 0) {
                        ROS_INFO("waiting for conveyor to advance a box to Q1...");
                }
        }

        //update box pose,  if possible
        if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
                ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
        }
        else {
                ROS_WARN("no box seen.  something is wrong! I quit!!");
                exit(1);
        }

        // if survive to here, then box is at Q1 inspection station;

        //inspect the box and classify all observed parts
        boxInspector.update_inspection(desired_models_wrt_world,
                                       satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                       misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                       orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                       part_indices_precisely_placed);
        ROS_INFO("orphaned parts in box: ");
        nparts = orphan_models_wrt_world.size();
        ROS_INFO("num parts seen in box = %d",nparts);
        for (int i=0; i<nparts; i++) {
                ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
        }

        if (boxInspector.get_bad_part_Q1(current_part)) {
                ROS_INFO("found bad part: ");
                ROS_INFO_STREAM(current_part<<endl);

                //Initializing necessary Variables
                Eigen::MatrixXd badPartWorld(4,4); //Bad part in reference to world

                //Convert the bad part pose to be in reference to the world and held in a 4x4
                //qCamToWorld = poseToTransform(boxInspector.getQualityCamera1Image().pose);
                badPartWorld = poseToTransform(current_part.pose.pose);

                //Find which model in orphan_models_wrt_world is closest to the pose of the bad part
                osrf_gear::Model badModel = findClosestModel(badPartWorld, orphan_models_wrt_world);


                //Converts model found above into "current_part" variable
                model_to_part(badModel, current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

                //use the robot action server to acquire and dispose of the specified part in the box:
                status = robotBehaviorInterface.pick_part_from_box(current_part);

                //use the robot action server to acquire and dispose of the specified part in the box:
        }

        //after removing the bad part, re-inspect the box:
        boxInspector.update_inspection(desired_models_wrt_world,
                                       satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                       misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                       orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                       part_indices_precisely_placed);
        ROS_INFO("orphaned parts in box: ");
        nparts = orphan_models_wrt_world.size();
        ROS_INFO("num parts seen in box = %d",nparts);
        for (int i=0; i<nparts; i++) {
                ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
        }

        //While there is still a part in the box, remove the next part.
        while (orphan_models_wrt_world.size() > 0) {
                //Converts next orphaned model into "current_part" variable
                model_to_part(orphan_models_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

                //use the robot action server to acquire and dispose of the specified part in the box:
                status = robotBehaviorInterface.pick_part_from_box(current_part);

                //Update information, important for us to update orphan_models_wrt_world
                boxInspector.update_inspection(desired_models_wrt_world,
                                               satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                               misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                               orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                               part_indices_precisely_placed);
        }

        return 0;
        //here's an oddity: this node runs to completion.  But sometimes, Linux complains bitterly about
        // *** Error in `/home/wyatt/ros_ws/devel/lib/shipment_filler/unload_box': corrupted size vs. prev_size: 0x000000000227c7c0 ***
        // don't know why.  But does not seem to matter.  If anyone figures this  out, please let me know.
}

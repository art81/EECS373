//Written by Andrew Tarnoff for EECS 373
//fill_order.cpp:

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

//For obtaining the order
#include <order_manager/order_manager.h>
#include <xform_utils/xform_utils.h>

const double COMPETITION_TIMEOUT = 500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

osrf_gear::Order order_;
bool gotNewOrder_ = false;
XformUtils xformUtils_;

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
int ans; //Used for easy break points

void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
        ROS_INFO_STREAM("Received order:\n" << *order_msg);

        osrf_gear::Order order;
        order_ = *order_msg;
        gotNewOrder_ = true;
}

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
        part.name = model.type;
        part.pose.pose = model.pose;
        part.location = location; //by default
}

//Finds which model in "models" is located closest to refModel
//This uses euclidean distance of x,y,z coordinates held in poses/transforms
osrf_gear::Model findClosestModel(osrf_gear::Model refModel, vector<osrf_gear::Model> models) {
        double minDist = -1;
        int minIdx = 0;

        double refX = refModel.pose.position.x;
        double refY = refModel.pose.position.y;
        double refZ = refModel.pose.position.z;

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

//Advances the box to the correct station.
//1 = quality station 1
//2 = quality station 2
//3 = loading dock
void moveBoxToStationConvertOrderToWorldAndReturnPose(int stationNum, BoxInspector &boxInspector, ConveyorInterface &conveyorInterface, geometry_msgs::PoseStamped &box_pose_wrt_world, osrf_gear::Shipment shipment) {
        //use conveyor action  server for multi-tasking
        int nprint = 0;
        int desBoxStatus;

        if (stationNum == 1) {
                //Q1
                conveyorInterface.move_new_box_to_Q1();
                desBoxStatus = conveyor_as::conveyorResult::BOX_SEEN_AT_Q1; //member function of conveyor interface to move a box to inspection station 1
        } else if (stationNum == 2) {
                //Q2
                conveyorInterface.move_box_Q1_to_Q2();
                desBoxStatus = conveyor_as::conveyorResult::BOX_SEEN_AT_Q2; //member function of conveyor interface to move a box to inspection station 2
        } else {
                //Loading Dock
                conveyorInterface.move_box_Q2_to_drone_depot(); //member function of conveyor interface to move a box to loading dock
                desBoxStatus = conveyor_as::conveyorResult::BOX_SENSED_AT_DRONE_DEPOT;
        }

        while (conveyorInterface.get_box_status() != desBoxStatus) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
                nprint++;
                if (nprint % 10 == 0) {
                        ROS_INFO("waiting for conveyor to advance a box to desired Location...");
                }
        }

        //update box pose,  if possible
        if(stationNum == 1 || stationNum == 2) {
                if (boxInspector.get_box_pose_wrt_world(stationNum, box_pose_wrt_world)) {
                        ROS_INFO_STREAM("box seen"<< endl);
                } else {
                        ROS_WARN("no box seen.  something is wrong! I quit!!");
                        exit(1);
                }

                //Convert order to world coords
                Eigen::Affine3d box_wrt_worldA, currP_wrt_boxA, currP_wrt_worldA;
                desired_models_wrt_world.clear();
                for(int i = 0; i < shipment.products.size(); i++) {
                        geometry_msgs::Pose currP = shipment.products[i].pose;

                        box_wrt_worldA = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world.pose);
                        currP_wrt_boxA = xformUtils_.transformPoseToEigenAffine3d(currP);
                        currP_wrt_worldA = box_wrt_worldA*currP_wrt_boxA;

                        //Copying transformed pose and type into desired_models_wrt_world
                        osrf_gear::Model newM;
                        newM.type = shipment.products[i].type;
                        newM.pose = xformUtils_.transformEigenAffine3dToPose(currP_wrt_worldA);

                        desired_models_wrt_world.push_back(newM);
                }
                order_.shipments[0] = shipment;
        }

        boxInspector.update_inspection(desired_models_wrt_world,
                                       satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                       misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                       orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                       part_indices_precisely_placed, stationNum);
}

//Removes all bad parts from a box at the given station number
void removeBadPartsFromBox(BoxInspector &boxInspector, RobotBehaviorInterface &robotBehaviorInterface, int stationNum) {
        inventory_msgs::Part current_part;
        bool isBadPart = false;

        if(stationNum == 1) {
                isBadPart = boxInspector.get_bad_part_Q1(current_part);
        } else {
                isBadPart = boxInspector.get_bad_part_Q2(current_part);
        }

        while (isBadPart) {
                ROS_INFO("found bad part: ");
                ROS_INFO_STREAM(current_part<<endl);

                //Initializing necessary Variables
                osrf_gear::Model current_part_model;
                current_part_model.pose = current_part.pose.pose;

                //Find which model in orphan_models_wrt_world is closest to the pose of the bad part
                osrf_gear::Model badModel = findClosestModel(current_part_model, orphan_models_wrt_world);

                //Converts model found above into "current_part" variable
                if(stationNum == 1) {
                        model_to_part(badModel, current_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                } else {
                        model_to_part(badModel, current_part, inventory_msgs::Part::QUALITY_SENSOR_2);
                }

                //grabs the part from the box:
                robotBehaviorInterface.pick_part_from_box(current_part);
                //Then discard it:
                robotBehaviorInterface.discard_grasped_part(current_part);

                //use the robot action server to acquire and dispose of the specified part in the box:
                //after removing the bad part, re-inspect the box:
                boxInspector.update_inspection(desired_models_wrt_world,
                                               satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                               misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                               orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                               part_indices_precisely_placed, stationNum);

                if(stationNum == 1) {
                        isBadPart = boxInspector.get_bad_part_Q1(current_part);
                } else {
                        isBadPart = boxInspector.get_bad_part_Q2(current_part);
                }
        }
}

//Puts all missing parts into box at given station number
bool putMissingPartsIntoBox(BoxInspector &boxInspector, RobotBehaviorInterface &robotBehaviorInterface, BinInventory &binInventory, int stationNum) {
        //Loops through all of the missing models and then puts them into the box in the correct location
        int num_missing = missing_models_wrt_world.size();
        int count = 0;
        inventory_msgs::Inventory current_inventory;

        while(num_missing > 0) {
                //Grab the part from the bin
                //Place it in the box at the location desired from missing_models_wrt_world
                ROS_INFO("Putting missing model into box");

                int n_missing_part = part_indices_missing[count];
                std::string part_name(desired_models_wrt_world[n_missing_part].type);
                bool part_in_inventory = true;
                int partnum_in_inv;

                inventory_msgs::Part pick_part, place_part;

                binInventory.update();
                binInventory.get_inventory(current_inventory);
                part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inv);

                if(!part_in_inventory) {
                        ROS_INFO("Part is not found, giving up");
                        return false;
                }

                if(stationNum == 1) {
                        model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_1);
                } else {
                        model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_2);
                }

                bool go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);

                if(!go_on) {
                        ROS_WARN("could not compute key pickup and place poses for this part source and destination");
                }

                //Pick part from Bin
                if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
                        ROS_INFO("pick failed");
                        go_on = false;
                        return false;
                }

                //Approach pose for placing part in box
                if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
                        ROS_WARN("could not move to approach pose");
                        go_on = false;
                        robotBehaviorInterface.discard_grasped_part(place_part);
                }

                //Place part
                if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
                        ROS_INFO("placement failed");
                        go_on = false;
                        return false;
                }

                robotBehaviorInterface.release_and_retract();

                boxInspector.update_inspection(desired_models_wrt_world,
                                               satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                               misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                               orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                               part_indices_precisely_placed, stationNum);
                num_missing = missing_models_wrt_world.size();
        }
        return true;
}

//Given a station number, this function will fix all of the misplaced parts in the box
void adjustMisplacedPartsInBox(BoxInspector &boxInspector, RobotBehaviorInterface &robotBehaviorInterface, int stationNum) {

        //Loops through all of the misplaced models and then puts them into the correct locations
        inventory_msgs::Part currP_des;
        inventory_msgs::Part currP_act;
        osrf_gear::Model currM_des;
        osrf_gear::Model currM_act;

        while(misplaced_models_actual_coords_wrt_world.size() > 0) {
                currM_des = misplaced_models_desired_coords_wrt_world[0];

                //Find which model in misplaced_models_actual_coords_wrt_world is closest to the pose of currP_des
                currM_act = findClosestModel(currM_des, misplaced_models_actual_coords_wrt_world);

                if(stationNum == 1) {
                        model_to_part(currM_des, currP_des, inventory_msgs::Part::QUALITY_SENSOR_1);
                        model_to_part(currM_act, currP_act, inventory_msgs::Part::QUALITY_SENSOR_1);
                } else {
                        model_to_part(currM_des, currP_des, inventory_msgs::Part::QUALITY_SENSOR_2);
                        model_to_part(currM_act, currP_act, inventory_msgs::Part::QUALITY_SENSOR_2);
                }

                robotBehaviorInterface.pick_part_from_box(currP_act); //Grasp the part
                robotBehaviorInterface.adjust_part_location_no_release(currP_act, currP_des); //Update its position
                robotBehaviorInterface.release_and_retract(); // release the part and retract the arm

                boxInspector.update_inspection(desired_models_wrt_world,
                                               satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                               misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                               orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                               part_indices_precisely_placed, stationNum);
        }
}

//Function that will completely fulfill an order based on boxInspector.update_inspection(...) at a given station
bool fixBoxAtStation(BoxInspector &boxInspector, RobotBehaviorInterface &robotBehaviorInterface, BinInventory &binInventory, int stationNum) {
        //Update description of environment
        boxInspector.update_inspection(desired_models_wrt_world,
                                       satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                       misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                       orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                       part_indices_precisely_placed, stationNum);

        //Remove bad parts at station
        removeBadPartsFromBox(boxInspector, robotBehaviorInterface, stationNum);
        //update_inspection was called at the end of this function call

        //Put all missing parts into correct spot in box
        if (!putMissingPartsIntoBox(boxInspector, robotBehaviorInterface, binInventory, stationNum)) {
                //if this function wanted to return false then so does this one
                return false;
        }
        //update_inspection was called at the end of this function call

        cout<<"Perturb parts now manually before it adjusts space, then press 1 to continue: "; //poor-man's breakpoint
        cin>>ans;

        //Update status after perturbing the parts
        boxInspector.update_inspection(desired_models_wrt_world,
                                       satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
                                       misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
                                       orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
                                       part_indices_precisely_placed, stationNum);

        //Adjust misplaced parts at quality station
        adjustMisplacedPartsInBox(boxInspector, robotBehaviorInterface, stationNum);
        //update_inspection was called at the end of this function call

        return true;
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "fill_order");
        ros::NodeHandle nh;

        //Initializing the Start Competition Service
        ros::ServiceClient startup_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        std_srvs::Trigger startup_srv;

        RobotBehaviorInterface robotBehaviorInterface(&nh);
        ConveyorInterface conveyorInterface(&nh);
        BinInventory binInventory(&nh);
        BoxInspector boxInspector(&nh);

        //Initializing the Drone Service
        ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
        osrf_gear::DroneControl drone_srv;

        //Order subscriber
        ros::Subscriber order_subscriber_ = nh.subscribe("/ariac/orders", 10, &order_callback);

        geometry_msgs::PoseStamped box_pose_wrt_world; //camera sees box, coordinates are converted to world coords

        //Starting up the competition
        startup_client.call(startup_srv);
        while(!startup_srv.response.success) {
                ROS_WARN("Failed to Start Competition");
                startup_client.call(startup_srv);
                ros::Duration(0.5).sleep();
        }
        ROS_INFO("Started the Competition");

        //Gets new order
        while(!gotNewOrder_) {
                //Waiting for new order to come in
                ros::spinOnce();
        }
        osrf_gear::Shipment shipment = order_.shipments[0];

        //Move box to Q1 and get the box's new pose in box_pose_wrt_world
        ROS_INFO("Getting a box into position at Q1: ");
        moveBoxToStationConvertOrderToWorldAndReturnPose(1, boxInspector, conveyorInterface, box_pose_wrt_world, shipment);

        //Fulfills the order at Q1
        fixBoxAtStation(boxInspector, robotBehaviorInterface, binInventory, 1);

        //Move box to Q2 and get the box's new pose in box_pose_wrt_world
        ROS_INFO("Getting a box into position at Q2: ");
        moveBoxToStationConvertOrderToWorldAndReturnPose(2, boxInspector, conveyorInterface, box_pose_wrt_world, shipment);

        //Fulfills the order at Q2
        fixBoxAtStation(boxInspector, robotBehaviorInterface, binInventory, 2);

        //Advances the box to the shipping dock
        moveBoxToStationConvertOrderToWorldAndReturnPose(3, boxInspector, conveyorInterface, box_pose_wrt_world, shipment);

        //Call the drone with the correct shipment name to get score and deliver box
        drone_srv.request.shipment_type = order_.shipments[0].shipment_type;
        drone_client.call(drone_srv);
        while(!drone_srv.response.success) {
                ROS_WARN("Failed to call the Drone");
                drone_client.call(drone_srv);
                ros::Duration(0.5).sleep();
        }
        ROS_INFO("Called the Drone: All Done");

        return 0;
}

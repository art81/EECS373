// Andrew Tarnoff for EECS 373

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <problem_set_3/sinCommanderAction.h>
#include <problem_set_3/sin_commander_server.h>
#include <iostream>

using namespace std;

// This function will be called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const problem_set_3::sinCommanderResultConstPtr& result) {
        bool finished = result->finished;
        ROS_INFO(" Server result for 'finished' parameter produced: %s", finished ? "true" : "false");
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "sin_commander_action_client");
        ros::NodeHandle nh;

        problem_set_3::sinCommanderGoal goal;

        // use the name of our server, which is: sin_commander_server
        actionlib::SimpleActionClient<problem_set_3::sinCommanderAction> action_client("sin_commander_server", true);

        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0));

        if (!server_exists) {
                ROS_WARN("could not connect to server; halting");
                return 0; // bail out; optionally, could print a warning message and retry
        }

        ROS_INFO("Connected to action server"); // if here, then we connected to the server;;

        double amplitude, freq, numCycles;

        cout << "Enter an Amplitude for commanded velocity sin wave: ";
        cin>>amplitude;
        cout << "Enter a Frequency for commanded velocity sin wave: ";
        cin>>freq;
        cout << "Enter the Number of Cycles for commanded velocity sin wave: ";
        cin>>numCycles;

        goal.amplitude = amplitude;
        goal.freq = freq;
        goal.numCycles = numCycles;

        action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired

        double elapTime = 0.0;
        while(!action_client.waitForResult(ros::Duration(0.01))) {
                elapTime += 0.01;
        }

        ROS_INFO(" Server took %f seconds to complete the goal", elapTime);

        return 0;
}

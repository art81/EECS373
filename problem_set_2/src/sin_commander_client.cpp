//By Andrew Tarnoff (EECS 373)
// sin_commander node:
#include <ros/ros.h>
#include <iostream>
#include <problem_set_2/FreqAmpService.h>

using namespace std;

int main(int argc, char **argv) {
        ros::init(argc, argv, "sin_commander"); //name this node
        ros::NodeHandle nh; // node handle
        ros::ServiceClient client = nh.serviceClient<problem_set_2::FreqAmpService>("sin_commander_service");
        problem_set_2::FreqAmpService srv;
        double amplitude, freq;

        cout << "Enter an Amplitude for commanded velocity sin wave (-1 to quit): ";
        cin>>amplitude;

        if(amplitude == -1)
                return 0;

        cout << "Enter a Frequency for commanded velocity sin wave: ";
        cin>>freq;

        srv.request.amplitude = amplitude;
        srv.request.freq = freq;
        client.call(srv);

        return 0; // should never get here, unless roscore dies
}

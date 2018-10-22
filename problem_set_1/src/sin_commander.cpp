//By Andrew Tarnoff (EECS 373)
// sin_commander node:
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<iostream>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander"); //name this node
    ros::NodeHandle nh; // node handle

    //publish a commanded velocity computer by the sin wave described by the user;
    ros::Publisher sin_vel_pub = nh.advertise<std_msgs::Float64>("vel_cmd", 1);

    //Obtaining sin wave description from user
    double amplitude = 0.0;
    double freq = 0.0;
    double PI = 4.0*atan(1); //Value for Pi since atan(1) = 0.25*PI
    double time = 0.0;
    std_msgs::Float64 sin_velocity; //Initialize variable to be published

    cout << "Enter an Amplitude for commanded velocity sin wave: ";
    cin>>amplitude;
    cout << "Enter a Frequency for commanded velocity sin wave: ";
    cin>>freq;

    double sample_rate = 20*freq; //Frequency Inputted by the user. Times 20 so that the loop will publish 20 values per period
    double dt_sin_commander = 1.0 / sample_rate; //Calculate dt from inputted frequency
    ros::Rate naptime(sample_rate); // use to regulate loop rate

    sin_velocity.data = 0.0; //initialize velocity to zero
    // enter the main loop: command the velocity based on the described sin wave
    // Publish this velocity to be useed by the minimal_controller
    // This loop will run at a frequency equivalent to that of the outputted sin wave
    while (ros::ok()) {
        sin_velocity.data = amplitude*sin(2*PI*freq*time);

        time += dt_sin_commander;
        sin_vel_pub.publish(sin_velocity); //Public the velocity derived from the described sin wave
        naptime.sleep(); // wait for remainder of specified period;
    }
    return 0; // should never get here, unless roscore dies
}

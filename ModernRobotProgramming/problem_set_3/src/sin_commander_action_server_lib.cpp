//Andrew Tarnoff for EECS 373

#include <problem_set_3/sinCommanderAction.h>
#include <problem_set_3/sin_commander_server.h>


// Callback being used by the sin commander action server
void SinCommanderServer::sinServerCallback(const actionlib::SimpleActionServer<problem_set_3::sinCommanderAction>::GoalConstPtr& goal) {
        isRunningCb = true;
        ROS_INFO("Called Sin Commander Callback in Server");

        ROS_INFO("Publishing CommandedSin");

        double freq = goal->freq;
        double amplitude = goal->amplitude;
        double numCycles = goal->numCycles;
        double PI = 4.0*atan(1); //Value for Pi since atan(1) = 0.25*PI
        double finalTime = (numCycles/freq); //Period times numCycles
        double time = 0.0;
        std_msgs::Float64 sin_velocity; //Initialize variable to be published

        double sample_rate = 20*freq; //Frequency Inputted by the user. Times 20 so that the loop will publish 20 values per period
        double dt_sin_commander = 1.0 / sample_rate; //Calculate dt from inputted frequency
        ros::Rate naptime(sample_rate); // use to regulate loop rate

        sin_velocity.data = 0.0; //initialize velocity to zero
        // enter the main loop: command the velocity based on the described sin wave
        // Publish this velocity to be useed by the minimal_controller
        // This loop will run at a frequency equivalent to that of the outputted sin wave
        while (ros::ok()) {
                if(time <= finalTime) {
                        sin_velocity.data = amplitude*sin(2*PI*freq*time);

                        time += dt_sin_commander;
                        sin_vel_pub_.publish(sin_velocity); //Public the velocity derived from the described sin wave
                        naptime.sleep(); // wait for remainder of specified period;
                } else {
                        result_.finished = true;
                        sin_as_.setSucceeded(result_);
                        break;
                }
        }
        isRunningCb = false;
}

//Constructor
SinCommanderServer::SinCommanderServer(ros::NodeHandle* nodehandle) :
        nh_(*nodehandle),
        sin_as_(nh_, "sin_commander_server", boost::bind(&SinCommanderServer::sinServerCallback, this, _1),false)
{
        ROS_INFO("In class constructor of SinCommanderServer");
        isRunningCb = false;
        sin_vel_pub_ = nh_.advertise<std_msgs::Float64>("vel_cmd", 1); //vel_cmd publisher

        sin_as_.start();
}

//Commands constant zeros at 100Hz when callback not activated. Timing done in "sin_commander_action_server.cpp"
void SinCommanderServer::publishZero(std_msgs::Float64 zero) {
        sin_vel_pub_.publish(zero);
}

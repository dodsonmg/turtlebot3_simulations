/*******************************************************************************
* Simulation Parameters
*******************************************************************************/
double wheel_speed_cmd[WHEEL_NUM] = {0.0, 0.0};
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

double loop_rate             = 1.;     // for turtlebot3 sim, this is originally 30Hz (1./30.)
double cmd_vel_timeout       = 10.0;    // for turtlebot3 sim, this is originally 1.0

float  odom_pose[3];
float  odom_vel[3];

/*
???:  why is this necessary?
used to set node params, which i removed.
push_back'ed into joint_states.name in .cpp
*/
std::string joint_states_name[2];

double last_position[WHEEL_NUM] = {0.0, 0.0};
double last_velocity[WHEEL_NUM] = {0.0, 0.0};

// assume TurtleBot3 Burger
double wheel_separation       = 0.287;
double turning_radius         = 0.1435;
double robot_radius           = 0.220;

double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                      0, 0.1,   0,   0,   0, 0,
                      0,   0, 1e6,   0,   0, 0,
                      0,   0,   0, 1e6,   0, 0,
                      0,   0,   0,   0, 1e6, 0,
                      0,   0,   0,   0,   0, 0.2};
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <thread>   

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"

#include "utility.h"

//Robot states
enum state_t {S_wait, S_travel, S_error};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//TODO: move inside class
void spinThread()
{
    ros::spin();
}

class Planner
{
public:

    Planner() : ac("move_base", true)
    {
        ROS_INFO("Initializing node...");
        odom_sub = n.subscribe("/odom", 1000, &Planner::odomCallback, this);
        vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        n.getParam("/planner/map_name", map_name);
        n.getParam("/planner/map_dimension_for_astar", map_dimension_astar);
        n.getParam("/planner/vertex_per_room", vertex_per_room);
        n.getParam("/planner/plan_file", plan_file);

        //spin the node in the background
        boost::thread spin_thread(&spinThread);
    }

    int init();
    void run();

private:

    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    ros::Publisher vel_pub;
    MoveBaseClient ac;         

    nav_msgs::Odometry turtle_odom;
    move_base_msgs::MoveBaseGoal goal;
    
    //CallBacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void resultCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    
    //Auxiliary functions
    void localize();
    vector<room_t> getBestRoomSequence();

    //Used to hold parameters value
    std::string map_name;
    std::string plan_file;
    int map_dimension_astar;
    int vertex_per_room;

    //private variable to keep track of robot actions
    vector<room_t> room_spec;
    vector<room_t> ordered_rooms;
    state_t pathState = S_wait;
    int room_count = 0;
    int vertex_count = 0;
    int rect_count = 0;
    bool rectangleCompleted = false;
};


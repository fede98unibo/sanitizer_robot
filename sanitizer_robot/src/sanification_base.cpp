#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <thread>   
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "std_srvs/Empty.h"


#include "nav_msgs/Odometry.h"

//#include "path_finder.h"
#include "utility.h"


using namespace std;
using namespace cv;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//Global Variables
Mat img;            //global varibale because both the mouse callback and create_plan function use this file , TODO: we can avoid this situation
int setpoint_counter = 0; //count the number of point written in the plan file
ofstream MyPlanFile;  //global varibale because both the mouse callback and create_plan function use this file , TODO: we can avoid this situation
nav_msgs::Odometry turtle_odom;
/*
@ Mouse callback function: used to plan a new set of rooms to be cleaned
*/
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          float setpoint_x = x * 0.05 -10;
          float setpoint_y = (384 - y)* 0.05 -10;

          //TODO: change 384 with image size
          
          cout << "Left button of the mouse is clicked - position (" << x*0.05 -10  << ", " << (384-y)*0.05 -10 << ")" << endl;
          
          cv::circle(img, cv::Point(x, y), 5, CV_RGB(255,0,0));

          // Write to the file
          MyPlanFile << setpoint_x << "\n";
          MyPlanFile << setpoint_y << "\n";

          setpoint_counter++;
          
          //show the image
     	imshow("Map window", img);
     }
}

/*
@ Function to crate a new plan 'file.txt' it returns the name of the file... TODO: return file path 
*/
string create_new_plan(string image_name)
{    
     ROS_INFO("Type the name of the plan file u want to create (.txt extension)");
     string filename;
     cin >> filename;
     // Create and open a text file
     MyPlanFile.open(filename);

     // Read image from file 
     img = imread(image_name);
     //if fail to read the image
     if ( img.empty() ) 
     { 
          cout << "Error loading the image" << endl;
          return {}; 
     }
     else
     {
          ROS_INFO("Select in the image the 4 EDGES of each room u want to clean");
          
          //Create a window
          namedWindow("Map window", WINDOW_NORMAL);

          //set the callback function for any mouse event
          setMouseCallback("Map window", CallBackFunc, NULL);

          //show the image
          imshow("Map window", img);

          ROS_INFO("To end pianification close the image !");

          // Wait until user press some key
          waitKey(0);

          MyPlanFile.close();
          
          return filename;
     }

 return {};
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    cout << "Receiving Odom" << endl;
     turtle_odom.pose.pose.position.x = msg->pose.pose.position.x;
     turtle_odom.pose.pose.position.y = msg->pose.pose.position.y;
     turtle_odom.pose.pose.position.z = msg->pose.pose.position.z;
}

/**************************************************************** MAIN ***************************************************************/

int main(int argc, char** argv)
{
     ros::init(argc, argv, "mission_planner");

     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);
     ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

     std::this_thread::sleep_for(1000ms);
     ROS_INFO("Initial Localization started");

     std_srvs::Empty empty_req;
     ros::service::call("/global_localization", empty_req);

     ros::Rate rate(10.0);

     geometry_msgs::Twist cmd_speed;
     cmd_speed.angular.z = 1;
     int deltaT = 0;

     while(ros::ok())
     {
         vel_pub.publish(cmd_speed);
         rate.sleep();
        
        deltaT ++;

        if(deltaT > 186)
        {
            cmd_speed.angular.z = 0;
            vel_pub.publish(cmd_speed);
            break;
        }
        
     }

/*** PLANNING MISSION ***/ 

     ROS_INFO("MISSION PLANNER \n");
     ROS_INFO("Type 'load' to load already created mission or type 'new' to create a new one:\n");

     //Get Command
     string cmd;
     cin >> cmd;
     string plan_file;

    if(cmd == "new"){
          plan_file = create_new_plan("map.pgm");
          ROS_INFO("New Plan File created\n");
    }

    else{
        ROS_INFO("Type the name of the plan file you want to load:\n");
        //Get File Name
        cin >> plan_file;
    }
    //Compute rooms specification from the input file
    vector<room_t> room_spec = room_details_from_plan_file(plan_file);
    print_rooms_spec(room_spec);

//***/

/*** DISCRETIZE MAP FOR A* ALGORITHM ***/

//load map image
Mat map = imread("map.pgm");

//discretize map with auxiliry function 
Mat map_resized = discretize_map(map);

//define grid, passed to Astar function
int grid[96][96];

//Transform mat in the right format to run Astar search and save the result in grid
for(int i =0; i< map_resized.rows-1; i++ )
{
     for(int j = 0; j < map_resized.cols-1; j++)
     {
          if(map_resized.at<char>(i,j)!=0)
          {
               grid[i][j]=1; //1 for unblocked cell
          }
          else{
               grid[i][j]=0;  //0 for blocked cell
          }
     }
}
//***/

//*** Optimal rooms order selection based on A* search ***///
vector<Point2i> rooms_centre_grid;
for(int i=0; i<room_spec.size(); i++){
     rooms_centre_grid.push_back(from_meters_to_pixel_coordinate(room_spec[i].center,4));
     cout << "room centre in the resized map" << rooms_centre_grid[i] << endl;
     cv::circle(map_resized, rooms_centre_grid[i], 2, CV_RGB(255,0,0));
}

//namedWindow("Resized Map", WINDOW_NORMAL);
//imshow("Resized Map", map_resized);

ros::Rate loop_rate(10);
int count = 0;
while (ros::ok())
  {
    if(count == 20)
        break;
    ros::spinOnce();
    loop_rate.sleep();
    count ++;
  }

//Compute Source Point 
Point2f src_point;
src_point.x  =  turtle_odom.pose.pose.position.x;
src_point.y = turtle_odom.pose.pose.position.x;

vector<int> optimalSequence = compute_best_sequence(room_spec, src_point, grid);

std::cout << "\nThe optimal sequence of rooms is:" << endl;
for(int k=0; k<room_spec.size(); k++)
{
    std::cout << "[" << optimalSequence[k] << "] ";
}

std::this_thread::sleep_for(10000ms);

/***/

//tell the action client that we want to spin a thread by default
MoveBaseClient ac("move_base", true); 

//wait for the action server to come up
while(!ac.waitForServer(ros::Duration(5.0))){
ROS_INFO("Waiting for the move_base action server to come up");
}

ROS_INFO("Starting Cleaning Sequence");

for(int i=0; room_spec.size(); i++)
{
    
    //Define goal msg for the Move-Base Server
    move_base_msgs::MoveBaseGoal room_goal;

    //Room Goal specification
    room_goal.target_pose.header.frame_id = "map";     //Set map frame to move from a room to another
    room_goal.target_pose.pose.position.x = room_spec[i].topLeft.x;
    room_goal.target_pose.pose.position.y = room_spec[i].topLeft.y;
    room_goal.target_pose.pose.orientation.w = 1; //TODO: set robot orientation also when changing room to avoid to stuck the robot
    room_goal.target_pose.header.stamp = ros::Time::now();

    //Send goal
    ROS_INFO("Next setpoint (%f,%f)",  room_goal.target_pose.pose.position.x,  room_goal.target_pose.pose.position.y);
    ac.sendGoal(room_goal);
    ac.waitForResult();

    bool clean_room = false; //Flag used to decide wheter a room should be cleaned or not

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        clean_room = true;    
        ROS_INFO("Room Reached");
    }
    else{
        clean_room = false;
        ROS_INFO("The base failed to reach the goal for some reason, try to go in the next room");
    }

    std::this_thread::sleep_for(1000ms); //wait after reaching a certain room

    if(clean_room == true)
    {   
        ROS_INFO("Start Room-Cleaning");
        for(int j=0; j < 4; j++)
        {
            //Check which vertex is source and destination
            Point2f source_vertex;
            Point2f dest_vertex;
            switch(j)
            {
                case 0:
                    cout << "case One" << endl;
                    source_vertex.x = room_spec[i].topLeft.x;
                    source_vertex.y = room_spec[i].topLeft.y;
                    dest_vertex.x = room_spec[i].bottomLeft.x;
                    dest_vertex.y = room_spec[i].bottomLeft.y;
                break;

                case 1:
                    cout << "case Two" << endl;
                    source_vertex.x = room_spec[i].bottomLeft.x;
                    source_vertex.y = room_spec[i].bottomLeft.y;
                    dest_vertex.x = room_spec[i].bottomRight.x;
                    dest_vertex.y = room_spec[i].bottomRight.y;
                break;

                case 2:
                    cout << "case Three" << endl;
                    source_vertex.x = room_spec[i].bottomRight.x;
                    source_vertex.y = room_spec[i].bottomRight.y;
                    dest_vertex.x = room_spec[i].topRight.x;
                    dest_vertex.y = room_spec[i].topRight.y;
                break;

                case 3:
                    cout << "case Four" << endl;
                    source_vertex.x = room_spec[i].topRight.x;
                    source_vertex.y = room_spec[i].topRight.y;
                    dest_vertex.x = room_spec[i].topLeft.x;
                    dest_vertex.y = room_spec[i].topLeft.y;
                break;
            }
            
            cout << "Source Vertex:" << source_vertex << endl;
            cout << "Destination Vertex:" << dest_vertex << endl;

            //Compute orientation
            nav_msgs::Odometry goal_spec = compute_orientation(source_vertex, dest_vertex);
            //Orientation goal specification
            move_base_msgs::MoveBaseGoal orientation_goal;
            orientation_goal.target_pose.header.frame_id = "map";
            orientation_goal.target_pose.pose.position.x = source_vertex.x;
            orientation_goal.target_pose.pose.position.y = source_vertex.y;
            orientation_goal.target_pose.pose.orientation = goal_spec.pose.pose.orientation;
            //Send Goal
            ac.sendGoal(orientation_goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Orinetation OK;");
            }
            else{
                ROS_INFO("The base failed to reach the goal for some reason");
                return 0;
            }

            std::this_thread::sleep_for(1000ms); //Wait after orienting the robot 

            //Compute Distance
            float distance = compute_forward_distance(source_vertex,dest_vertex);
            cout << "Next Vertex Distance:" << distance << endl;
            //Forward Goal Specification
            move_base_msgs::MoveBaseGoal position_goal;
            position_goal.target_pose.header.frame_id = "map";
            position_goal.target_pose.pose.orientation = goal_spec.pose.pose.orientation;
            position_goal.target_pose.pose.position.x = dest_vertex.x;
            position_goal.target_pose.pose.position.y = dest_vertex.y;

            //Send Goal
            ac.sendGoal(position_goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("position goal ok");
            }
            else{
                ROS_INFO("The base failed to reach the goal for some reason");
                return 0;
            }

            std::this_thread::sleep_for(1000ms); //Wait after reaching a new vertex
        }
    }

}
return 0;

}

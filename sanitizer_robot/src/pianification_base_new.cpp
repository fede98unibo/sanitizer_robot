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

#include "nav_msgs/Odometry.h"

//#include "path_finder.h"
#include "utility.h"
#include "astar_search.h"

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
     turtle_odom.pose.pose.position.x = msg->pose.pose.position.x;
     turtle_odom.pose.pose.position.y = msg->pose.pose.position.y;
     turtle_odom.pose.pose.position.z = msg->pose.pose.position.z;
}

/**************************************************************** MAIN ***************************************************************/

int main(int argc, char** argv)
{
     ros::init(argc, argv, "mission_planner");

     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);

     //tell the action client that we want to spin a thread by default
     MoveBaseClient ac("move_base", true); 

/*** PLANNING MISSION ***/ 

     ROS_INFO("MISSION PLANNER \n");
     ROS_INFO("type 'new' to create a new mission plan file or type 'load' to load already created plan");

     string cmd;
     cin >> cmd;
     string plan_file;
     
     
     if(cmd == "new")
     {
          plan_file = create_new_plan("map.pgm");
          ROS_INFO("pianification finished");
     }
    else if (cmd=="load")
     {
          ROS_INFO("Type the name of the plan file you want to load\n");
          cin >> plan_file;
          vector<room_t> room_spec = room_details_from_plan_file(plan_file);
          print_rooms_spec(room_spec);
     }

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


/*
New Method for discretizing map- not very good until now

for(int i=0; i<map.rows-1; i++)
{
     if(i % 4 == 0)
     {
          for(int j=0; j<map.cols-1; j++)
          {
               if(j % 4 == 0)
               {
                    Vec3b & pixel = map.at<Vec3b>(Point(i,j));

                    if((pixel[0]==255) && (pixel[1]==255) && (pixel[2]==255))
                    {
                         grid1[i/4][j/4] = 0;
                    }
                    else 
                    grid1[i/4][j/4] = 1;
               }
          }
     }
}
*/

/*
cout << "CONVERTED IMAGE:"  << endl;


print_mat(map_resized,2);


cout << "GRID MAP:"  << endl;

for(int i=0; i< map_resized.rows-1; i++)
{
     for(int j=0;j<map_resized.rows-1;j++)
     {
          cout<<" "<< grid[i][j] <<"";
     }
     cout << "\n";
}*/

//***/

//*** Optimal rooms order selection based on A* search ***///

//compute rooms centre for Astar Search
vector<Point2i> rooms_centre = compute_rooms_center(plan_file, setpoint_counter);


vector<Point2i> rooms_centre_grid;
for(int i=0; i<rooms_centre.size(); i++){
     rooms_centre_grid.push_back(from_meters_to_pixel_coordinate(rooms_centre[i],4));
     cout << "room centre in the resized map" << rooms_centre_grid[i] << endl;
     cv::circle(map_resized, rooms_centre_grid[i], 2, CV_RGB(255,0,0));
}

namedWindow("Resized Map", WINDOW_NORMAL);
imshow("Resized Map", map_resized);

//Compute Source Point 
Point2f src_point;
src_point.x  =  turtle_odom.pose.pose.position.x;
src_point.y = turtle_odom.pose.pose.position.x;
Point2i src_resized = from_meters_to_pixel_coordinate(src_point,4);
Pair src = make_pair(src_resized.x, src_resized.y);

//Compute destination Points
vector<Pair> dst_pair;
for(int i=0; i<rooms_centre_grid.size(); i++){
     dst_pair.push_back(make_pair(rooms_centre_grid[i].x, rooms_centre_grid[i].y));
}

//Initialize the cost vector to go from source to destination
vector<int> cost;
//Compute cost
for(int i = 0; i<dst_pair.size(); i++)
{
     cost.push_back(aStarSearch(grid, src, dst_pair[i]));
     cout << "cost["<<i<<"] = " << cost[i] << endl;
}

/***/

//wait for the action server to come up
while(!ac.waitForServer(ros::Duration(5.0))){
ROS_INFO("Waiting for the move_base action server to come up");
}

//Define goal msg for the Move-Base Server
move_base_msgs::MoveBaseGoal goal;
goal.target_pose.header.frame_id = "map";     //Set map frame to move from a room to another

//Check which is the number of 
std::vector<int>::iterator result;
result = std::min_element(cost.begin(), cost.end());
int next_room = std::distance(cost.begin(), result);

std::cout << "The next room with the smaller path cost is: " << next_room << '\n';

//Position
goal.target_pose.pose.position.x = rooms_centre[next_room].x;
goal.target_pose.pose.position.y = rooms_centre[next_room].y;
goal.target_pose.pose.orientation.w = 1;

ROS_INFO("next setpoint (%f,%f)",  goal.target_pose.pose.position.x,  goal.target_pose.pose.position.y);

//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.stamp = ros::Time::now();

ROS_INFO("Sending goal");
ac.sendGoal(goal);

ac.waitForResult();

if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
ROS_INFO("setpoint reached");
}
else
{
     ROS_INFO("The base failed to reach the goal for some reason");
     return 0;
}

//Room Cleaning 

//*** ROTATE THE ROBOT ***//
move_base_msgs::MoveBaseGoal orientation_goal;
orientation_goal.target_pose.header.frame_id = "base_link";
//Compute orientation
Point2f point1, point2;
point1.x = turtle_odom.pose.pose.position.x;
point1.y = turtle_odom.pose.pose.position.y;
point2.x = turtle_odom.pose.pose.position.x;
point2.y = turtle_odom.pose.pose.position.y-1;
nav_msgs::Odometry goal_spec = compute_orientation(point1, point2);
orientation_goal.target_pose.pose.position.x = 0;
orientation_goal.target_pose.pose.position.y = 0;
orientation_goal.target_pose.pose.orientation = goal_spec.pose.pose.orientation;

ac.sendGoal(orientation_goal);
ac.waitForResult();
if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
ROS_INFO("Orinetation OK;");
}
else
{
     ROS_INFO("The base failed to reach the goal for some reason");
     return 0;
}

//*** MOVE STRAIGHTFORWARD TO THE NEXT SETPOINT ***//
move_base_msgs::MoveBaseGoal position_goal;
position_goal.target_pose.header.frame_id = "base_link";
Point2f source_vertex, dest_vertex;
source_vertex.x = turtle_odom.pose.pose.position.x;
source_vertex.y = turtle_odom.pose.pose.position.y;

position_goal.target_pose.pose.orientation.w = compute_forward_distance(source_vertex,dest_vertex);
position_goal.target_pose.pose.position.x = 0.5;

ac.sendGoal(position_goal);
ac.waitForResult();
if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
ROS_INFO("position goal ok");
}
else
{
     ROS_INFO("The base failed to reach the goal for some reason");
     return 0;
}

return 0;

}

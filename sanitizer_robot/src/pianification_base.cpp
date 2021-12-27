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
          string temp;
          ifstream Plan(plan_file);
          while(getline(Plan,temp))
          {
               setpoint_counter++;
          }
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

/*
int x = (rooms_centre[0].x + 10)/(0.05);
int y = (-(rooms_centre[0].y + 10)/(0.05) + 384) ;
int x1 = (rooms_centre[1].x + 10)/(0.05);
int y1 = (-(rooms_centre[1].y + 10)/(0.05) + 384) ;

cv::circle(map, Point2i(x,y), 5, CV_RGB(255,0,0));
cv::circle(map, Point2i(x1,y1), 5, CV_RGB(255,0,0));

namedWindow("rooms centre Map", WINDOW_NORMAL);
imshow("rooms centre Map", map);
*/

vector<Point2i> rooms_centre_grid;
for(int i=0; i<rooms_centre.size(); i++){
     rooms_centre_grid.push_back(from_meters_to_pixel_coordinate(rooms_centre[i],4));
     cout << "room centre in the resized map" << rooms_centre_grid[i] << endl;
     cv::circle(map_resized, rooms_centre_grid[i], 2, CV_RGB(255,0,0));
}

//namedWindow("Resized Map", WINDOW_NORMAL);

//imshow("Resized Map", map_resized);

// waitKey(0);
// Source is the left-most bottom-most corner

Point2f src_point;
src_point.x  =  turtle_odom.pose.pose.position.x;
src_point.y = turtle_odom.pose.pose.position.x;
Point2i src_resized = from_meters_to_pixel_coordinate(src_point,4);
Pair src = make_pair(src_resized.x, src_resized.y);

vector<Pair> dst_pair;
for(int i=0; i<rooms_centre_grid.size(); i++){
     dst_pair.push_back(make_pair(rooms_centre_grid[i].x, rooms_centre_grid[i].y));
}

vector<int> cost;

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

move_base_msgs::MoveBaseGoal goal;

// the frame is fixed
goal.target_pose.header.frame_id = "map";     

ifstream MyFile(plan_file);

bool mission_completed = false;

while(mission_completed==false)
{    
     std::vector<int>::iterator result;
     result = std::min_element(cost.begin(), cost.end());

     int next_room = std::distance(cost.begin(), result);

     if(cost[next_room]==1000)
          return 0;

     std::cout << "The next room with the smaller path cost is: " << next_room << '\n';

     //Position
     goal.target_pose.pose.position.x = rooms_centre[next_room].x;
     goal.target_pose.pose.position.y = rooms_centre[next_room].y;

     //quaternion
     goal.target_pose.pose.orientation.w = 1.0;

     ROS_INFO("next setpoint (%f,%f)",  goal.target_pose.pose.position.x,  goal.target_pose.pose.position.y);

     //we'll send a goal to the robot to move 1 meter forward
     goal.target_pose.header.stamp = ros::Time::now();

     ROS_INFO("Sending goal");
     ac.sendGoal(goal);

     ac.waitForResult();

     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
     ROS_INFO("setpoint reached");
     cost[next_room]=1000;
     }
     else
     {
          ROS_INFO("The base failed to reach the goal for some reason");
          break;
     }

}   
MyFile.close();


return 0;

}

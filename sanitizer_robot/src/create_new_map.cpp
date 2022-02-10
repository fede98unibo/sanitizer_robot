#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <thread>   


using namespace std;
using namespace cv;


//Global Variables
Mat img;            //global varibale because both the mouse callback and create_plan function use this file , TODO: we can avoid this situation
int setpoint_counter = 0; //count the number of point written in the plan file
ofstream MyPlanFile;  //global varibale because both the mouse callback and create_plan function use this file , TODO: we can avoid this situation

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


/**************************************************************** MAIN ***************************************************************/

int main(int argc, char** argv)
{

/*** PLANNING MISSION ***/ 

     ROS_INFO("MISSION PLANNER \n");

     string plan_file = create_new_plan("map.pgm");
     
     ROS_INFO("New Plan File created\n");
 
return 0;

}

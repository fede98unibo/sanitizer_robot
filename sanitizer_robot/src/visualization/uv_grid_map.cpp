#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <chrono>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "uv_utility.h"

#define Power 0.1
#define cleanTreshold 10.0
#define MAP_ODOM_X_DISPLACEMENT -0.14
#define MAP_ODOM_Y_DISPLACEMENT -0.18

using namespace grid_map;

/* Globals variables */
grid_map::GridMap map({"Layer1"});
grid_map::GridMap map1({"Layer1"});
nav_msgs::Odometry turtle_odom;
float map_size_x;
float map_size_y;
float map_position_x;
float map_position_y;
float map_resolution;
bool mapOk = false;

/*** Callbacks ***/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
     //Update position of the robot
     turtle_odom.pose.pose.position.x = msg->pose.pose.position.x;
     turtle_odom.pose.pose.position.y = msg->pose.pose.position.y;
     turtle_odom.pose.pose.position.z = 0; //always zero
}

void gridMapCallback(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("Received Grid Map");
  //Save a copy of the received map
  grid_map::GridMapRosConverter::fromOccupancyGrid(msg, {"Layer1"}, map);

  mapOk = true;
  map_size_x = map.getSize()(0);
  map_size_y = map.getSize()(1);
  map_position_x = map.getPosition().x();
  map_position_y = map.getPosition().y();
  map_resolution = map.getLength().x()/ map.getSize()(0);

}
/***/


int main(int argc, char** argv)
{
  // Initialize node, publisher and subscriber.
  ros::init(argc, argv, "UV_visualization_node");
  ros::NodeHandle nh("~");
  ros::Subscriber grid_map_sub_ = nh.subscribe("/map", 1, gridMapCallback);
  ros::Subscriber odom_sub_ = nh.subscribe("/odom", 1, odomCallback);
  ros::Publisher grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  //Define new map
  map1.setFrameId("map");
  map1.setGeometry(Length(19.2, 19.2), 0.05, Position(-0.4,-0.4));

  //Initialize grid map with all zeros
  grid_map::Matrix& data = map1["Layer1"];
  for (grid_map::GridMapIterator iterator(map1); !iterator.isPastEnd(); ++iterator){
    const int i = iterator.getLinearIndex();
    data(i) = 0;}

  Position currentPosition; //store current robot position for the circle iterator
  const double radius = 19.2; //radius for the circle iterator
  const int obstacle_value = 100; //value for a cell obstructed by a wall

  
  float distance; 
  bool obstacleFound = false; //if true an obstacle is found between two cell
  float tempPower = 0;
  long int deltaT; //loop time in [ms]
  float deltaT_s;  //loop time in [s]

  auto End = std::chrono::steady_clock::now();
  auto Start = std::chrono::steady_clock::now();

  ros::Rate rate(2.0);

  while (nh.ok()) {

    if(mapOk == true)
    {
        End = std::chrono::steady_clock::now();

        deltaT = std::chrono::duration_cast<std::chrono::milliseconds>(End-Start).count();
        deltaT_s = float(deltaT)/1000;

        Start = std::chrono::steady_clock::now();

        ROS_INFO("Delta T: %f", deltaT_s);
        //Circle iterator specifications
        Position center(turtle_odom.pose.pose.position.x-MAP_ODOM_X_DISPLACEMENT, turtle_odom.pose.pose.position.y-MAP_ODOM_Y_DISPLACEMENT);
        
        for (grid_map::CircleIterator iterator(map, center, radius); !iterator.isPastEnd(); ++iterator) {

        //get cell position 
        map.getPosition(*iterator, currentPosition);

        obstacleFound = false;
        
        if(sqrt(pow(center.x()-currentPosition.x(),2) + pow(center.y()-currentPosition.y(),2)) >= 0.11)
        {
            //These are in the gridmap reference frame because we check obstacles in a map transleted of 0.4 0.4 wrt the map reference frame.
            Index start(round(-(center.x()- map_position_x)/map_resolution)+map_size_x/2,round(-(center.y()-map_position_y)/map_resolution)+map_size_y/2);
            Index end(round(-(currentPosition.x()- map_position_x)/map_resolution)+map_size_x/2,round(-(currentPosition.y()-map_position_y)/map_resolution)+map_size_y/2);

            //Line iterator
            for (grid_map::LineIterator iterator1(map, start, end); !iterator1.isPastEnd(); ++iterator1) {

            //Obstacle Check
            if(map.at("Layer1", *iterator1) == obstacle_value)
                obstacleFound = true;
            
            //If obstacle is found exit the line iterator
            if(obstacleFound == true)
                break;
            }   

            //Update Energy
            if((obstacleFound == false) && (map.at("Layer1", *iterator) != obstacle_value) )
            {
                distance = pow(turtle_odom.pose.pose.position.x-currentPosition.x(),2) + pow(turtle_odom.pose.pose.position.y-currentPosition.y(),2);

                tempPower = map1.atPosition("Layer1", currentPosition) + (Power*(deltaT_s))/distance;

                if(tempPower >= cleanTreshold){
                  map1.atPosition("Layer1", currentPosition) = cleanTreshold;
                }
                else{
                  map1.atPosition("Layer1", currentPosition) = tempPower;
                }
            }
        }
      }    

    }

    ros::Time time = ros::Time::now();
    map1.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map1, message);
    grid_map_pub_.publish(message);
    //SpinOnce to run callbacks
    ros::spinOnce();
    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}
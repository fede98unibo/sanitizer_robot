#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <chrono>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "UV_energy_utility.h"

#define Power 0.1

using namespace grid_map;


grid_map::GridMap map({"Layer1"});
grid_map::GridMap map1({"Layer1"});
nav_msgs::Odometry turtle_odom;
bool mapOk = false;

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
  grid_map::GridMapRosConverter::fromOccupancyGrid(msg, {"Layer1"}, map);
  mapOk = true;
  ROS_INFO("Received map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());
}


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

  float distance;
  bool obstacleFound = false;
  float tempPower = 0;
  auto End = std::chrono::steady_clock::now();
  auto Start = std::chrono::steady_clock::now();
  long int deltaT;
  float deltaT_s;

  //Initialize grid map with all zeros
  grid_map::Matrix& data = map1["Layer1"];
  for (grid_map::GridMapIterator iterator(map1); !iterator.isPastEnd(); ++iterator){
    const int i = iterator.getLinearIndex();
    data(i) = 0;}


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
        Position center(turtle_odom.pose.pose.position.x-0.14, turtle_odom.pose.pose.position.y-0.18);
        double radius = 19.2;

        for (grid_map::CircleIterator iterator(map, center, radius); !iterator.isPastEnd(); ++iterator) {

        //get cell position
        Position currentPosition;
        map.getPosition(*iterator, currentPosition);

        obstacleFound = false;

        //These are in the gridmap reference frame because we check obstacles in a map transleted of 0.4 0.4 wrt the map reference frame.
        Index start(round(-(center.x()+0.4)/0.05)+192,round(-(center.y()+0.4)/0.05)+192);
        Index end(round(-(currentPosition.x()+0.4)/0.05)+192,round(-(currentPosition.y()+0.4)/0.05)+192);

        //Line iterator
        for (grid_map::LineIterator iterator1(map, start, end); !iterator1.isPastEnd(); ++iterator1) {

        //Obstacle Check
        if(map.at("Layer1", *iterator1) == 100.0)
            obstacleFound = true;
        
        //If obstacle is found exit the line iterator
        if(obstacleFound == true)
            break;
        }   

        //Update Energy
        if((obstacleFound == false) && (map.at("Layer1", *iterator) != 100.0) )
        {
            distance = pow(turtle_odom.pose.pose.position.x-currentPosition.x(),2) + pow(turtle_odom.pose.pose.position.y-currentPosition.y(),2);

            tempPower = map1.atPosition("Layer1", currentPosition) + (Power*(deltaT_s))/distance;

            if(tempPower >= 10){
              map1.atPosition("Layer1", currentPosition) = 10.0;
            }
            else{
              map1.atPosition("Layer1", currentPosition) = tempPower;
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
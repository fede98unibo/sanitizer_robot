#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#include "nav_msgs/Odometry.h"

using Eigen::MatrixXd;

//Homoogeneous tranformation from map to robot frame
Eigen::Matrix4f compute_homogeneous_tranformation_from_pose(const nav_msgs::Odometry::ConstPtr& robotPose)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0,3) = robotPose -> pose.pose.position.x;
    T(1,3) = robotPose -> pose.pose.position.y;
    T(2,3) = robotPose -> pose.pose.position.z;

    Eigen::Quaternionf q;
    q.x() = robotPose -> pose.pose.orientation.x;
    q.y() = robotPose -> pose.pose.orientation.y;
    q.z() = robotPose -> pose.pose.orientation.z;
    q.w() = robotPose -> pose.pose.orientation.w;
    Eigen::Matrix3f R = q.toRotationMatrix();

    T.block(0,0,3,3) = R;

    return T;
}

//Find the index representing the closest angle in the laser scanner measurement to the angle between the current cell and the robot frame
int find_closest(float angle, float angleIncrement)
{
    /*int currentDistance = 0, previousDistance = 0;
    int count = 0;
    float scan_number = 2*M_PI/angleIncrement;

    previousDistance = abs(angle - angleIncrement*count);

    for( count=1; count <= int(scan_number); count++)
    {
        currentDistance = abs(angle - angleIncrement*count);

        if(currentDistance >= previousDistance)
            return count-1;
 
        previousDistance = currentDistance;
    }
    return count-1;
    */

    const int scan_number = round(2*M_PI/angleIncrement);
    std::vector<float> array;
    int j = 0;
    for(int i=0; i<scan_number; i++)
    {
        array.push_back(angleIncrement*i);
    }

    float closest_value = array[0];
    float subtract_result = abs(closest_value - angle);

    for(int i=0; i<scan_number; i++)
    {
        if(abs(array[i] - angle) < subtract_result)
        {
            subtract_result = abs(array[i] - angle);
            closest_value = array[i];
            j = i;
        }
    }

return j;

}
#include "planner.h"

/****** Callbacks ******/

void Planner::resultCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if(vertex_count==4)
        { 
            if(rect_count < room_spec[room_count].resizedVertices.size())
            {
                vertex_count = 0;
                rect_count++;
            }
                    
            else if(rect_count == room_spec[room_count].resizedVertices.size() && (rectangleCompleted == false))
            {
                rectangleCompleted = true;
            }

            else if(rect_count == room_spec[room_count].resizedVertices.size() && (rectangleCompleted == true)) //center reached
            {
                room_count++;
                rect_count = 0;
                vertex_count = 0;
                rectangleCompleted = false;
            }
        }

        else 
            vertex_count++;

        pathState = S_wait;
    }

    else
        pathState = S_error;
}

void Planner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    turtle_odom.pose.pose.position.x = msg->pose.pose.position.x;
    turtle_odom.pose.pose.position.y = msg->pose.pose.position.y;
    turtle_odom.pose.pose.position.z = msg->pose.pose.position.z;
}

/*******/

/****** Private Methods ******/

vector<room_t> Planner::getBestRoomSequence()
{
    vector<room_t> orderedRooms;

    Mat map = imread(map_name);
    Mat map_resized = discretize_map(map);

    //define grid, passed to Astar function
    int grid[96][96];

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

    //Compute Source Point 
    Point2f src_point;
    src_point.x  =  turtle_odom.pose.pose.position.x;
    src_point.y = turtle_odom.pose.pose.position.y;

    vector<int> optimalSequence = compute_best_sequence(room_spec, src_point, grid);

/*
    std::cout << "\nThe optimal sequence of rooms is:" << endl;
    for(int k=0; k<room_spec.size(); k++)
    {
        std::cout << "[" << optimalSequence[k] << "] ";
    }
*/
    for(int kk=0; kk<room_spec.size(); kk++)
        orderedRooms.push_back(room_spec[optimalSequence[kk]]);

    return orderedRooms;
}


void Planner::localize()
{
    geometry_msgs::Twist cmd_speed;

    std_srvs::Empty empty_req;
    ros::service::call("/global_localization", empty_req); // spread particles

    int deltaT = 0;
    int localization_phase = 0;
    bool localization_end = false;

    ros::Rate rate(10);

    while(ros::ok())
    {
        switch(localization_phase)
        {
            case 0:
            {
                cmd_speed.angular.z = 1;
                deltaT++;

                //Stop the robot after one round
                if(deltaT > (4*M_PI/cmd_speed.angular.z)*10){
                    cmd_speed.angular.z = 0;
                    deltaT=0;
                    localization_phase++;
                }

                vel_pub.publish(cmd_speed);
            }
            break;

            case 1:
            {
                cmd_speed.linear.x = 0.2;
                deltaT++;

                if(deltaT > 5*10){
                    cmd_speed.linear.x = 0;
                    deltaT=0;
                    localization_phase++;
                }

                vel_pub.publish(cmd_speed);
            }
            break;

            case 2:
            {
                cmd_speed.angular.z = 1;
                deltaT++;

                //Stop the robot after one round
                if(deltaT > (4*M_PI/cmd_speed.angular.z)*10){
                    cmd_speed.angular.z = 0;
                    deltaT=0;
                    localization_end = true;
                }

                vel_pub.publish(cmd_speed);
            }
            break;
        }

        if(localization_end == true)
            break;

        rate.sleep();
    }
}

/******/

/****** Core node's function ******/

int Planner::init()
{
    //Initial Localization
    localize();

    //TODO: Check the result of the localization, if not ok repeat

    //Get rooms informations
    room_spec = room_details_from_plan_file(plan_file);
    if(room_spec.empty()){
        ROS_ERROR("Something went wrong while scanning the plan file");
        return -1;}
    else{
        print_rooms_spec(room_spec);
        ordered_rooms = getBestRoomSequence();
        return 0;}

    return -1;
}

void Planner::run()
{
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");}

    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        switch(pathState)
        {
            case S_wait:
            {
                if(rectangleCompleted == true)
                {
                    //Room Goal specification
                    goal.target_pose.header.frame_id = "map";     //Set map frame to move from a room to another
                    goal.target_pose.pose.position.x = ordered_rooms[room_count].center.x;
                    goal.target_pose.pose.position.y = ordered_rooms[room_count].center.y;
                    goal.target_pose.pose.orientation.w = 1; //TODO: set robot orientation also when changing room to avoid to stuck the robot
                    goal.target_pose.header.stamp = ros::Time::now();
                }
                else if(rect_count == 0)
                {
                    //Room Goal specification
                    goal.target_pose.header.frame_id = "map";     //Set map frame to move from a room to another
                    goal.target_pose.pose.position.x = ordered_rooms[room_count].vertices[(vertex_count%4)].x;
                    goal.target_pose.pose.position.y = ordered_rooms[room_count].vertices[(vertex_count%4)].y;
                    goal.target_pose.pose.orientation.w = 1; //TODO: set robot orientation also when changing room to avoid to stuck the robot
                    goal.target_pose.header.stamp = ros::Time::now();
                }
                else
                {
                    //Room Goal specification
                    goal.target_pose.header.frame_id = "map";     //Set map frame to move from a room to another
                    goal.target_pose.pose.position.x = ordered_rooms[room_count].resizedVertices[rect_count-1][(vertex_count%4)].x;
                    goal.target_pose.pose.position.y = ordered_rooms[room_count].resizedVertices[rect_count-1][(vertex_count%4)].y;
                    goal.target_pose.pose.orientation.w = 1; //TODO: set robot orientation also when changing room to avoid to stuck the robot
                    goal.target_pose.header.stamp = ros::Time::now();
                }
                                
                // Need boost::bind to pass in the 'this' pointer
                ac.sendGoal(goal,
                            boost::bind(&Planner::resultCallback, this, _1, _2),
                            MoveBaseClient::SimpleActiveCallback(),
                            MoveBaseClient::SimpleFeedbackCallback());

                ROS_INFO("Switching to state: S_travel");
                pathState = S_travel;
            }
            break;

            case S_travel:
                //DO some stuff while traveling
            break;

            case S_error:
                ROS_ERROR("Move Base fails to reach the goal");
                ros::shutdown();
            break;
        }

        loop_rate.sleep();
    }

    ROS_INFO("Progam Succesfully Ended");
    ros::shutdown();
}

/******/

/****** MAIN ******/
int main (int argc, char **argv)
{
  ros::init(argc, argv, "path_planner");
  /*Planner my_planner;
  my_planner.run();

  ros::spin();*/

  auto my_planner = std::make_shared<Planner>();

  ros::Duration duration(10);
  duration.sleep();

  if(my_planner->init()==0)
    my_planner->run();
  else
    ROS_ERROR("Cannot intialize the planner for some reason... exiting");

  return 0;
}

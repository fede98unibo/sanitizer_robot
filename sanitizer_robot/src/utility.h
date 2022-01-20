#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <bits/stdc++.h>
#include "astar_search.h"

using namespace std;
using namespace cv;

struct room_t {
     vector<Point2f> vertices;
     vector<vector<Point2f>> resizedVertices;
     Point2f center;
     };


vector<vector<Point2f>> vertices_smaller_rectangles(vector<Point2f> vertices_original, Point2f center){

ROS_INFO("Computing sub rectagles");

vector<vector<Point2f>> new_vertices;
vector<Point2f> vertices_room;
Point2f new_vertex;

// definition of "perfect" concentric rectangles starting from the original vertex
// furthest from the center of the room
int i = 0;
int k = 0;
int j = 0;
float temp_distance = 0;
float max_distance = 0;
float step_size = 0.75;

for(i=0; i<vertices_original.size(); i++){
	temp_distance = sqrt(pow(center.x - vertices_original[i].x,2) + pow(center.y - vertices_original[i].y,2));
	if (temp_distance > max_distance){
		max_distance = temp_distance;
		j=i;
	}
}

ROS_INFO(" max distance: %f, furthest vertex: %d", max_distance, j);

// k number of concentric rectangles I'll create
// if k = 0 no rectangles are created

if (max_distance <= 1){
	return{};
}

k = floor(max_distance/step_size);
ROS_INFO("k:%d", k);

ROS_INFO("furthest vertex (%f, %f)", vertices_original[j].x, vertices_original[j].y);

for (i = 1; i <= k; i++){
	
     if(j ==0){
		//new top left
		new_vertex.x = vertices_original[j].x + i*(step_size*sqrt(2)/2);
		new_vertex.y = vertices_original[j].y - i*(step_size*sqrt(2)/2);
		vertices_room.push_back(new_vertex);
		
		//new top right
		new_vertex.x = center.x + abs(center.x - new_vertex.x);
		vertices_room.push_back(new_vertex);

		//new bottom right
		new_vertex.y = center.y - abs(center.y - new_vertex.y);
		vertices_room.push_back(new_vertex);

		//new bottom left
		new_vertex.x = center.x - abs(center.x - new_vertex.x);
		vertices_room.push_back(new_vertex);}

	if(j==1){
		//new top right
		new_vertex.x = vertices_original[j].x - i*(step_size*sqrt(2)/2);
		new_vertex.y = vertices_original[j].y - i*(step_size*sqrt(2)/2);
		vertices_room.push_back(new_vertex);
		
		//new bottom right
		new_vertex.y = center.y - abs(center.y - new_vertex.y);
		vertices_room.push_back(new_vertex);

		//new bottom left
		new_vertex.x = center.x - abs(center.x - new_vertex.x);
		vertices_room.push_back(new_vertex);

		//new top left
		new_vertex.y = center.y + abs(center.y - new_vertex.y);
		vertices_room.insert(vertices_room.begin(), new_vertex);}
	
	if(j==2){
		//new bottom right
		new_vertex.x = vertices_original[j].x - i*(step_size*sqrt(2)/2);
		new_vertex.y = vertices_original[j].y + i*(step_size*sqrt(2)/2);
		vertices_room.push_back(new_vertex);

		//new bottom left
		new_vertex.x = center.x - abs(center.x - new_vertex.x);
		vertices_room.push_back(new_vertex);

		//new top left
		new_vertex.y = center.y + abs(center.y - new_vertex.y);
		vertices_room.insert(vertices_room.begin(), new_vertex);
		
		//new top right
		new_vertex.x = center.x + abs(center.x - new_vertex.x);
		vertices_room.insert(vertices_room.begin()+1, new_vertex);}
	
	if(j==3){
		//new bottom left
		new_vertex.x = vertices_original[j].x + i*(step_size*sqrt(2)/2);
		new_vertex.y = vertices_original[j].y + i*(step_size*sqrt(2)/2);
          vertices_room.push_back(new_vertex);
		
		//new top left
		new_vertex.y = center.y + abs(center.y - new_vertex.y);
		vertices_room.insert(vertices_room.begin(), new_vertex);
          
          //new top right
		new_vertex.x = center.x + abs(center.x - new_vertex.x);
		vertices_room.insert(vertices_room.begin()+1, new_vertex);
	
		//new bottom right
		new_vertex.y = center.y - abs(center.y - new_vertex.y);
		vertices_room.insert(vertices_room.begin()+2, new_vertex);}

for(int kk=0; kk<vertices_room.size();kk++){
     ROS_INFO("vertices %d (%f,%f)", kk , vertices_room[kk].x, vertices_room[kk].y);
}

new_vertices.push_back(vertices_room);
vertices_room = vector<Point2f>();
}


return(new_vertices);

}


/*
@Description: - This function compute all the rooms details (i.e. vertex and center) based on the informations
                contained in the plan file of interesr. !!! This works only for rectangular shaped rooms.

@Inputs: - The path to the location of the plan file
         - The destination point
         
@Return: - A dynamic array of 'room_t' containing all the informations.
*/
vector<room_t> room_details_from_plan_file(string filePath)
{
     std::ifstream PlanFile(filePath); //open the file where vertex coordinate are stored

     vector<room_t> room;
     room_t temp_room;
     Point2f temp_point;

     int rooms_counters = 0;
     int vertex_counter = 0;

     //To hold temporary data read in the file 
     std::string temp_x;
     std::string temp_y;

     bool file_end = false; //True if plan file end

     while(file_end == false)
     {
          vertex_counter++;

          if(getline(PlanFile,temp_x))
          {
               getline(PlanFile,temp_y); //The number of point is always even
          }
          else
          {
               file_end = true;
               std::cout << "Plan File Scanning Completed \n" << endl;
               break;
          }

          temp_point.x = stof(temp_x);
          temp_point.y = stof(temp_y);
          temp_room.vertices.push_back(temp_point);          

          if(vertex_counter == 4)
          {
               //Rooms comleted -> reset vertex
               vertex_counter=0;

               //compute room center
               temp_room.center.x = (temp_room.vertices[0].x + temp_room.vertices[1].x + temp_room.vertices[2].x + temp_room.vertices[3].x)/4;
               temp_room.center.y = (temp_room.vertices[0].y + temp_room.vertices[1].y + temp_room.vertices[2].y + temp_room.vertices[3].y)/4;

               room.push_back(temp_room);

               temp_room.vertices = std::vector<Point2f>();
          }
     }

for(int i=0; i<room.size(); i++)
{
room[i].resizedVertices = vertices_smaller_rectangles(room[i].vertices, room[i].center);  
}

return room;    
}

void print_rooms_spec(vector<room_t> rooms)
{
     cout << "The number of rooms is:" <<  rooms.size() << endl; 
     
     for(int i=0; i < rooms.size(); i++)
     {
          cout << "--------------" << endl;
          cout << "ROOM" << i << endl;
          cout << "room center: "  << rooms[i].center << endl;
          for(int j=0; j<4;j++)
          {
               cout << "Vertex "<< j << ":" << rooms[i].vertices[j] << endl;
          }
     }
/*
     for(int k=0; k<rooms.size(); k++)
     {
          cout << "the number of resized vertices in room " << k << "is " << rooms[k].resizedVertices[0].size() << endl;
          for(int p=0; p<rooms[k].resizedVertices[p].size(); p++)
          {
               ROS_INFO("Resized vertex %d: (%f,%f)", p , rooms[k].resizedVertices[k][p].x, rooms[k].resizedVertices[k][p].y) ;
          }
     }
*/
}


/*
@Core function to create a grid map from the original map to run A* search algorithm
*/
Mat discretize_map(Mat map)
{
     //initialize mat for resized image
     Mat map_resized;

     //convert to grayscale
     cv::cvtColor(map, map, cv::COLOR_RGB2GRAY);

     //Binary threshold       
     threshold(map,map,230,255, THRESH_BINARY);
     //imshow("thresholded image", map);

     // EROSION: To avoid holes in the walls during resize step
     int morph_size = 2; 
     Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size +2, 2 * morph_size+2), Point(morph_size, morph_size));
     erode(map, map, element, Point(-1, -1), 1);

     //imshow("eroded image", map);

     //Find contours
     vector<vector<cv::Point>> contours;
     vector<cv::Vec4i> hierarchy;
     cv::findContours(map, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);    

     //detect all contours with area smaller than a TH and try to delete them by drawing very width contours
     for(int i=0; i<contours.size(); i++)
     {	     
          if((cv::contourArea(contours[i])< 250))
          {
               drawContours( map, contours, i, 255, 25, LINE_8, hierarchy, 0 );
          }
     }
     // Dilation: To decrease the thikness of walls that was augmented during the erosion
     morph_size = 1; 
     element = getStructuringElement(MORPH_RECT, Size(2 * morph_size +2, 2 * morph_size+2), Point(morph_size, morph_size));
     dilate(map, map, element, Point(-1, -1), 1); // comment if u want to reduce map size to 48x48

     //Resize image: 4:1
     resize(map, map_resized, Size(96,96), INTER_LINEAR);
     //Final threshold to ensure that all is black and white ( dont understand why but ok)
     threshold(map_resized,map_resized,250,255, THRESH_BINARY);
     //Create a window
     //namedWindow("discretized map window", WINDOW_NORMAL);
     //show the image
     //imshow("discretized map window", map_resized);

     return map_resized;
}

/*
@Description: - This function compute the corresponding point if the input point in a scaled coordinate system.
                It is useful when we need coordinate in scaled map used by A_star.

@Inputs: - The input point
         - The scale factor of the map of interest
         
@Return: - The corresponding point coordinate
*/
Point2i from_meters_to_pixel_coordinate(Point2f point, int scale)
{
     Point2i pixel_point;
     pixel_point.x = (point.x + 10)/(0.05*scale);
     pixel_point.y = (-(point.y + 10)/(0.05) + 384) /scale;

     return pixel_point;
}

/*
@Description: - This function the quaternion associated with the orinetation of the line passing through 
                the source and destination point. The quaternion is compute in the Map Reference Frame

@Inputs: - The source point
         - The destination point
         
@Return: - The resulting quaternion
*/
nav_msgs::Odometry compute_orientation(Point2f src, Point2f dst)
{

     //angle from angular coefficent
     float yaw = atan2(dst.y-src.y,dst.x-src.x);
     //cout << "Yaw:" << yaw << endl;

     const float pitch = 0;
     const float roll = 0;

     //quaternion from angle
     double cy = cos(yaw*0.5);
     double sy = sin(yaw*0.5);
     double cp = cos(pitch*0.5);
     double sp = sin(pitch*0.5);
     double cr = cos(roll*0.5);
     double sr = sin(roll*0.5);

     nav_msgs::Odometry q;

     q.pose.pose.orientation.w = cr*cp*cy + sr*sp*sy;
     q.pose.pose.orientation.x = sr*cp*cy - cr*sp*sy;
     q.pose.pose.orientation.y = cr*sp*cy + sr*cp*sy;
     q.pose.pose.orientation.z = cr*cp*sy - sr*sp*cy;

     return q;
}

/*
@Description: - This function compute the distance between two given point

@Inputs: - The source point
         - The destination point
         
@Return: - The distance between source and destination point
*/
float compute_forward_distance(Point2f src, Point2f dst)
{
     float distance = sqrt(pow(src.x - dst.x,2) + pow(src.y - dst.y, 2));
     return distance;
}

/*
@Description: - This function compute the factorial of a givel number

@Inputs: - nn -> !nn
         
@Return: - the result of the factorial of nn
*/
int factorial(int nn)
{
	int i;
	int fact=1;
	
	for(i=1; i<=nn; i++){
		fact = fact*i;
	}

	return fact;
}

/*
@Description: - This function is used to find the best sequence of rooms to be cleaned, i.e. the one that minimize the cost of the total path.
                In order to compute it we use A_start algorithm to find the cost of passing from a room to another, moreover all the possible 
                combination of rooms sequences are considered.

@Inputs: - A vector of 'room_t' elements containg all the informations about the rooms
         - The starting position of the robot 
         - The grid map used by A_star algorithm

@Return: - A vector of int containing the best sequence of rooms. TODO: Maybe is better to return an array of room_t in the optimal sequence
*/
vector<int> compute_best_sequence(vector<room_t> roomSpec, Point2f sourcePosition, int grid[96][96])
{
     int n_perm = factorial(roomSpec.size());
     int n_rooms = roomSpec.size();

     int partial_cost = 0;
     int permutations_set[n_perm][n_rooms]; //fixed array to hold all possible permutation of the rooms 

     // array rooms such that if there are 4 rooms, construction of the array {0, 1, 2, 3}
    int rooms[n_rooms];
    for (int i=0; i<n_rooms; i++){
        rooms[i]=i;
    } 
    sort(rooms, rooms + n_rooms);

// permutations_set is a two dimensional array where each row is a permutation of rooms to be sanitized
    int k=0;
    do 
    {
        for(int i=0; i<n_rooms; i++)
          {
               permutations_set[k][i] = rooms[i];
          }
        k++;
    } while (next_permutation(rooms, rooms + n_rooms));

// now I need to compute the sum of the five a* results for each permutation
// in order to be more clear: given the permutation 3-2-0-1, I have to sum the following paths
// spawn of the robot --> centre room3 || centre room3 --> centre room2
// centre room2 --> centre room 0 || centre room0 ---> centre room1
// these n_perm summations are collected into the vector cost
vector<Point2i> rooms_centre_grid;
for(int i=0; i<roomSpec.size(); i++){
     rooms_centre_grid.push_back(from_meters_to_pixel_coordinate(roomSpec[i].center,4));
}

//Compute Source Point for the transition from the robot spawn to the first room (in particular center of the first room)
Point2i src_resized = from_meters_to_pixel_coordinate(sourcePosition,4); //TODO: use robot position
Pair src = make_pair(src_resized.x, src_resized.y);

//Compute destination Points
vector<Pair> dest_pair;
for(int i=0; i<n_rooms; i++){
     dest_pair.push_back(make_pair(rooms_centre_grid[i].x, rooms_centre_grid[i].y));
}

//cost associated with each permutation 
vector<int> cost;
//Compute cost for the n_perm possible paths
for(int i = 0; i<n_perm; i++)
{
	int j = 0;
     //std::cout << "\nPath from source to room [" << permutations_set[i][j] << "]" << endl;
     partial_cost = 0;
	
	partial_cost = aStarSearch(grid, src, dest_pair[permutations_set[i][j]]);
    	for (j=0; j<(n_rooms-1); j++)
	{
		//std::cout << "Path from room [" << permutations_set[i][j] << "] to room [" << permutations_set[i][j+1]<< "]" << endl;
          partial_cost = partial_cost + aStarSearch(grid, dest_pair[permutations_set[i][j]], dest_pair[permutations_set[i][j+1]]);
	}
	cost.push_back(partial_cost);
     std::cout << "The cost of the sequence given by the permutation number " << i << "is: " << partial_cost << endl; 
}

// the order of rooms (permutation) the robot will follow is the one that generates the lowest cost,
// namely, that has the lowest element in the vector cost just constructed
// the following part should be adapted
std::vector<int>::iterator result;
result = std::min_element(cost.begin(), cost.end());
int bestPermutation = std::distance(cost.begin(), result);
//Orderd best sequence
vector<int> bestSequence;
for(int k=0; k<n_rooms; k++){
     bestSequence.push_back(permutations_set[bestPermutation][k]);
}

return bestSequence;
}
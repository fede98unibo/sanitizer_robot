#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

using namespace std;
using namespace cv;

struct room_t {
     cv::Point2f topLeft;
     cv::Point2f topRight;
     cv::Point2f bottomLeft;
     cv::Point2f bottomRight;
     cv::Point2f center;
     };


vector<room_t> room_details_from_plan_file(string filePath)
{
     std::ifstream PlanFile(filePath); //open the file where vertex coordinate are stored

     vector<room_t> room;
     room_t temp_room;

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
               getline(PlanFile,temp_y);
          }
          else
          {
               file_end = true;
               std::cout << "Plan File Scanning Completed" << endl;
               break;
          }

          switch(vertex_counter)
          {
               case 1:
                    temp_room.topLeft.x = stof(temp_x);
                    temp_room.topLeft.y = stof(temp_y);
               break;

               case 2:
                    temp_room.topRight.x = stof(temp_x);
                    temp_room.topRight.y = stof(temp_y);
               break;               
               case 3:
                    temp_room.bottomLeft.x = stof(temp_x);
                    temp_room.bottomLeft.y = stof(temp_y);
               break; 
               case 4:
                    temp_room.bottomRight.x = stof(temp_x);
                    temp_room.bottomRight.y = stof(temp_y);
               break; 
          }

          if(vertex_counter == 4)
          {
               //Rooms comleted -> reset vertex
               vertex_counter=0;

               //compute room center
               temp_room.center.x = (temp_room.topLeft.x + temp_room.topRight.x + temp_room.bottomLeft.x + temp_room.bottomRight.x)/4;
               temp_room.center.y = (temp_room.topLeft.y + temp_room.topRight.y + temp_room.bottomLeft.y + temp_room.bottomRight.y)/4;

               room.push_back(temp_room);
          }
     }
return room;    
}

void print_rooms_spec(vector<room_t> rooms)
{
     for(int i=0; i < rooms.size(); i++)
     {
          cout << "--------------" << endl;
          cout << "ROOM" + i << endl;
          cout << "room center: "  << rooms[i].center << endl;
          cout << "Top Left:" << rooms[i].topLeft << endl;
          cout << "Top Right:" << rooms[i].topRight << endl;
          cout << "Bottom Left:" << rooms[i].bottomLeft << endl;
          cout << "Bottom Right:" << rooms[i].bottomRight << endl;
     }
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

     //imshow("contours drawed", map);

     // Dilation: To decrease the thikness of walls that was augmented during the erosion
     morph_size = 1; 
     element = getStructuringElement(MORPH_RECT, Size(2 * morph_size +2, 2 * morph_size+2), Point(morph_size, morph_size));
     dilate(map, map, element, Point(-1, -1), 1); // comment if u want to reduce map size to 48x48

     //Resize image: 4:1
     resize(map, map_resized, Size(96,96), INTER_LINEAR);
     //Final threshold to ensure that all is black and white ( dont understand why but ok)
     threshold(map_resized,map_resized,250,255, THRESH_BINARY);




     //Create a window
     namedWindow("discretized map window", WINDOW_NORMAL);
     //show the image
     imshow("discretized map window", map_resized);

     return map_resized;
}

/*
@ Given the plan_file path and the number of setpoint in the path compute the rooms centre in [m] wrt the map centre
*/
vector<Point2i> compute_rooms_center(string plan_file, int setpoint_counter)
{
    ifstream MyFile(plan_file); //open the file where vertex coordinate are stored

    //instantiate vector for rooms vertex and center
     vector<Point2i> rooms_centre;
     vector<Point2i> rooms_vertex;
     int rooms_counters = 0;
     int vertex_counter = 0;

    //string to save what we read from file 
     std::string k;

     for(int i = 0; i < setpoint_counter; i++)
     {
          vertex_counter++; //increment vertex counter once for evry loop

          Point2i new_vertex; //instantiate new empty vertex

          //get vertex coordinate from file and save them in the empty vertex
          getline(MyFile,k);
          new_vertex.x = stof(k);
          getline(MyFile,k);
          new_vertex.y = stof(k);

          rooms_vertex.push_back(new_vertex);

        //every four vertex a room is created --> compute room center
          if(vertex_counter % 4 == 0)
          {
              Point2i new_centre;
              new_centre.x = 0;
              new_centre.y = 0;

              for(int j = vertex_counter-4; j < vertex_counter; j++ )
               {
                    new_centre.x = new_centre.x + rooms_vertex[j].x;
                    new_centre.y = new_centre.y + rooms_vertex[j].y;
               }

               new_centre.x = new_centre.x/4;
               new_centre.y = new_centre.y/4;

               rooms_centre.push_back(new_centre);

               rooms_counters++; //keep track of the number of rooms that we want to clean
          }
     }   
     
     cout << "Room1 center:" << rooms_centre[0] << endl;
     cout << "Room2 center:" << rooms_centre[1] << endl;


     cout << "the number of rooms is:" << rooms_counters << endl;

     if(rooms_counters > 0) 
        return rooms_centre;

    else
        return {};

}

/*
@Print matrix with pixel value from grayscale image
*/
void print_mat(Mat mat, int prec)
{
     for(int i =0; i< mat.size().height; i++)
     {
          cout<<"[";
          for(int j =0; j< mat.size().width;j++)
          {
               cout << setprecision(prec) << mat.at<char>(i,j);
               if(j != mat.size().width-1)
                    cout << ",";
               else 
                    cout<<"]" << endl;
          }
     }
}

/*
@Function to transform coordinate in [m] wrt to map center to pixel coordinate.
@With the 'scale' paeam u can also specify the compression of the original map 
*/
Point2i from_meters_to_pixel_coordinate(Point2f point, int scale)
{
     Point2i pixel_point;
     pixel_point.x = (point.x + 10)/(0.05*scale);
     pixel_point.y = (-(point.y + 10)/(0.05) + 384) /scale;

     return pixel_point;
}

/*
@
*/
nav_msgs::Odometry compute_orientation(Point2f src, Point2f dst)
{

     //angle from angular coefficent
     float yaw = atan2(dst.y-src.y,dst.x-src.x);
     cout << "Yaw:" << yaw << endl;

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

float compute_forward_distance(Point2f src, Point2f dst)
{
     float distance = sqrt(pow(src.x - dst.x,2) + pow(src.y - dst.y, 2));
     return distance;
}
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream> 

#define MAP_ROWS = 96
#define MAP_COLS = 96

using namespace std;
using namespace cv;

enum state_t {OPEN,CLOSED,EXPANSION};

// A structure to hold the necessary parameters
struct cell {
    state_t current_state;
    // Row and Column index of its parent
    Point2i parent;
    // g = path cost , k = smallest value of h since the given cell was placed in the open list
    double g, k;
};

class PathFinder 
{
public:

    PathFinder(Mat input_map)
    {
        map = input_map;
    }

    ~PathFinder();

    // A Function to find the shortest path between a given source cell to a destination cell according
    // to A* Search Algorithm
    vector<Point2i> DstarSearch();
    

private:

    //Check if a given cell is valid 
    bool isValid(Point2i cell_coordinate);

    //Check if a given cell is blocked (walls, obstacles, etc..)
    bool isBlocked(Point2i cell_coordinate);

    //Check if a given cell is the destination
    bool isDestination(Point2i cell_coordinate_src, Point2i cell_coordinate_dst);

    //Compute Path Cost for a given cell
    float computeHValue(Point2i src_point, Point2i dst_point);

    //Compute the D* path from src to dst
    void tracePath(Point2i src_point, Point2i dst_point){}

    //Grid map
    Mat map;

    //Cell map rappresenting the grid map
    cell cell_map[MAP_COLS][MAP_ROWS];


};


    #include<path_finder.h>

    bool PathFinder::isValid(Point2i cell_coordinate)
    {
        return(cell_coordinate.x>0) && (cell_coordinate.y>0) && (cell_coordinate.x< rows) && (cell_coordinate.y<map.cols);
    }

    bool PathFinder::isBlocked(Point2i cell_coordinate)
    {
        if(map.at<int>(cell_coordinate.x,cell_coordinate.y) == 0)
            return 1;
        else 
            return 0;
    }

    bool isDestination(Point2i cell_coordinate_src, Point2i cell_coordinate_dst)
    {
        if((cell_coordinate_src.x == cell_coordinate_dst.x) && (cell_coordinate_src.y == cell_coordinate_dst.y))
            return true;
        else
            return false;
    }

    float PathFinder::computeHValue(Point2i src_point, Point2i dst_point)
    {
        return sqrt(pow(src_point.x-dst_point.x,2)+pow(src_point.y-src_point.y,2));
    }
    
    vector<Point2i> PathFinder::DstarSearch(Point2i src, Point2i dst)
    {
        if(isValid(src) == false){
            ROS_ERROR(this->getlogger(),"Source Point not valid");
            return{};
        }

        if(isValid(dst) == false){
            ROS_ERROR(this->getlogger(),"Destination Point is not valid");
            return{};
        }

        if((isBlocked(src) == true) || (isBlocked(dst) == true)){
            ROS_ERROR(this->getlogger(),"Destination or Source is blocked");
            return{};
        }

        if(isDestination(src,dst) == true){
            ROS_WARN(this->getlogger(),"Source and Destination Coincide");
            return{}; //TODO: return the actual source point
        }

        //Initialize cell_map
        for(int i=0; i<MAP_ROWS; i++)
        {
            for(int j=0; j<MAP_COLS; j++)
            {
                cell_map[i][j].g = 0;
                cell_map[i][j].k = 0;
                cell_map[i][j].current_state = OPEN;
            }
        }

        //Expand all 8 neighbornhood of the cells starting from destination
        int i = dst_point.x;
        int j = dst_point.y;

        Point2i current_cell;

        bool foundDst = false;


        while(foundDst == true)
        {

        /*** 1st neighborn ***/
        current_cell.x = i;
        current_cell.y = j;
        if(isValid(current_cell) == true)
        {
            if(isDestination(current_cell) == true)
            {
                cout << "Destination Found" << endl;
                cell_map[current_cell.x][current_cell.y].parent = 
            }
        }



        }

    }
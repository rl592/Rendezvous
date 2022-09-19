#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Bool.h"
// #include <nav_msgs/MapMetaData.h>
// #include <stdint.h>
// #include <sstream>
#include "helper_funcs.hh"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sstream>
#include <stdlib.h>  
#include <algorithm>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// #include <move_base_msgs/MoveBaseAction.h>
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define radius 20 
#define thresh 15

// using namespace cv;
// using namespace std;

ros::Publisher pub_node;
// ros::Publisher pub_edge;

bool start = false;
int width=800, height=800;
float resolution = 0.05; 
float origin[2] = {-20,-20};

uint8_t ** gridmap;
uint8_t premap[800][800]; 
uint8_t prepremap[800][800]; 
uint8_t preprepremap[800][800]; 
uint8_t prepreprepremap[800][800]; 
uint8_t curmap[800][800]; 
string robot_name;
int cnt = 0;
int curpose[2] = {399, 399};
int range = 5 * 20;

int getIndexOfNode(int x1, int y1, int (*node)[2], int rows){
    for (int i=0; i<rows; i++){        
        if (distance(x1, node[i][0], y1, node[i][1])< 0.001)
            return i;
    }
    return -1;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

    for (int i = 0; i < height; i++)  delete[] gridmap[i];
    delete[] gridmap; 

    width = msg->info.width;
    height = msg->info.height;

    gridmap = new uint8_t*[height];
    for (int i = 0; i < height; i++)   gridmap[i] = new uint8_t[width];

    int n = 0; 
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++) {
                gridmap[j][i] = msg->data[n];
            n++;
        }
    }


    std_msgs::Float64 x;
    std_msgs::Float64 y;
    x.data = msg->info.origin.position.x;
    y.data = msg->info.origin.position.y;
    origin[0] = msg->info.origin.position.x;
    origin[1] = msg->info.origin.position.y;
    resolution = msg->info.resolution;
    std_msgs::Float32 reso;
    reso.data = msg->info.resolution;
    cout << robot_name <<": Map updated: "<< height<<"x" <<width <<" ("<< x.data<<","<< y.data <<"), resolution: "<< reso.data<< endl;



}



void move_cb(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data){

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++) {
                if ((int)sqrt((curpose[0] - i) * (curpose[0] - i) + (curpose[1] - j) * (curpose[1] - j)) <= range)
                    curmap[i][j] = gridmap[i][j];
                else
                    curmap[i][j] = 255;
            }
        }


    bool animate = true;
    Mat img = Mat(height, width, CV_8UC1, &curmap);

    // if input segfaults, change from thresh_binary to thresh_binary_inv and vice versa
    threshold( img, img, 240, 254, THRESH_BINARY_INV );

    std::cout << "stage 7" << std::endl;


    
    // Preprocess image by blurring
    Mat img_blur;

    GaussianBlur(img, img_blur, Size(3,3), 3000, 3000);

    // dilate(img_blur, img_blur, cv::Mat(), cv::Point(-1,-1));
    dilate(img_blur, img_blur, cv::Mat(), cv::Point(-1,-1));
    dilate(img_blur, img_blur, cv::Mat(), cv::Point(-1,-1));


    Mat edges;
    Canny(img_blur, edges, 100, 200, 3, false);

    // 1. Important floorplan (thresholded)
    // imshow( "input_img", img );
    // imwrite("outputimgs/1.jpg", img);

    // // 2. Blurred img
    // imshow( "blurred_im", img_blur );
    // imwrite("outputimgs/2.jpg", img_blur);

    // // 3. Canny edge detected image 
    // imshow("Canny edge detection", edges);
    // imwrite("outputimgs/3.jpg", edges);


    vector<vector<Point> > contours;
    vector<Point2f> all_pts;


    vector<Vec4i> hierarchy;
    RNG rng(12345);
    
    // 1. try thresholding map
    // 2. try getting rid of mask
    // 3. more smoothing/filtering
    // 4. try labeling obstacles 

    // SOLUTION: Get rid of ending mask, threshold at beginning instead 
    // now, get the graph looking pretty with trimmed points

    // RETR EXTERNAL FINDS EXTERNAL BORDERS OF FLOORPLAN 
    // CHAIN_APPROX_NONE DOESN'T COMPILE CONTOUR POINTS ON A STRAIGHT LINE/DIAGONAL 
    
    findContours( edges, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE, Point(0, 0) );
    // / Draw contoursGauss

    Mat mask = Mat::zeros( img.size(), CV_8U );

    // Keep a copy around
    Mat img_orig = img.clone();

    // Rectangle to be used with Subdiv2D


    Size size = img.size();
    Rect rect(0, 0, size.width, size.height);    

    // Create an instance of Subdiv2D
    Subdiv2D subdiv(rect);

    // Create a vector of points.
    vector<Point2f> points;

    // loop through contours array of arrays, compile into points vector
    for( int i = 0; i < contours.size(); i++ ) {
        for( int j = 0; j < contours[i].size(); j++ ) {

            Point2f closest_pt = find_closest(points, contours[i][j].x, contours[i][j].y);
            // check if outside distance range instead of out of either the x or y ranges???
            // use find dist formula here too? 
            // or need to optimize placement on corners, etc 
            
            if ( distance(closest_pt.x, contours[i][j].x, closest_pt.y, contours[i][j].y ) > thresh ) {
                points.push_back(Point2f(contours[i][j].x, contours[i][j].y));
            }
        }
    }    


    // Insert points into subdiv
    for( vector<Point2f>::iterator it = points.begin(); it != points.end(); it++)
    {
        subdiv.insert(*it);
        // Show animation
        if (animate)
        {
            Mat img_copy = img_orig.clone();
            waitKey(100);
        }
    }


    // Allocate space for Voronoi Diagram
    // Mat img_voronoi = Mat::zeros(img.rows, img.cols, CV_8UC3);

    /*stores the output*/
    // cv::Mat output;    

    /*compute the bitwise and of input arrays*/
    /*and store them in output array*/

    // bitwise_and(img, img, output, mask);

    // Draw Voronoi diagram
    float avg_d = 0.0;
    size_t num_vert = 0; 

    std::map<cv::Point, vector<Point>, ComparePoints> adj_map; 

    Rect rect2 = boundingRect(points); 

    Point pt1, pt2;
    pt1.x = rect2.x;
    pt1.y = rect2.y;
    pt2.x = rect2.x + rect2.width;
    pt2.y = rect2.y + rect2.height;
    // Draws the rect in the original image and show it

    draw_voronoi( img, subdiv, rect2, &avg_d, &num_vert, points, adj_map, 20 );
    // std::cout << "test3: " << std::endl; 


    // std::cout << "adj_map.size(): " << adj_map.size() << std::endl; 


    // std::cout << "avg_d: " << avg_d << std::endl;         


    // cvtColor(output, coutput, COLOR_GRAY2BGR);

    // // create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
    // cv::Mat coutput(img.size(), CV_8UC3);

        // // convert grayscale to color image
        // cv::cvtColor(img, coutput, COLOR_GRAY2BGR);        

        // for( vector<Point2f>::iterator it = points.begin(); it != points.end(); it++)
        // {
        //     draw_point(coutput, *it);
        // }    

    // COMMENT OUT TO get rid of rectangle generation on image 
    Mat img_out = img.clone();

    rectangle(img, pt1, pt2, 255, 5);

    // 8. Show floorplan bounding rectangle
    // imshow("rectangle", img );   
    // imwrite("outputimgs/8.jpg", img); 


    cv::putText(img_out, "avg_distance: " + to_string(avg_d), cv::Point(220, 25), cv::FONT_HERSHEY_COMPLEX_SMALL, .75, cv::Scalar(255,255,255), 1, cv::LINE_AA);
    cv::putText(img_out, "num_vertices: " + to_string(num_vert), cv::Point(220, 55), cv::FONT_HERSHEY_COMPLEX_SMALL, .75, cv::Scalar(255,255,255), 1, cv::LINE_AA);

    // Show results.
    imshow( robot_name + " :Voronoi Diagram", img_out);
    imwrite("info/G/Voronoi Diagram_" + to_string(cnt) +".jpg", img_out); 
    cnt ++;


    // float node_set[node_list.size() * 2];
    // for ( int i = 0; i < node_list.size(); i++ ) { 
    //     float nodex = node_list[i].y * resolution + origin[1];
    //     float nodey = node_list[i].x * resolution + origin[0] ;
    //     nodes.data.push_back(nodex);
    //     nodes.data.push_back(nodey);
    //     // std::cout << "asd1: " << node_list[i] << std::endl; 
    // }

    std_msgs::Float32MultiArray nodesToPub;
    // std_msgs::Float32MultiArray edgesToPub;
    std::map<cv::Point, vector<Point>, ComparePoints>::iterator it;
    float edges_matrix[adj_map.size()][adj_map.size()];
    int nodeList[adj_map.size()][2];

    for (int i = 0; i < adj_map.size(); ++i)
    {
        for (int j = 0; j < adj_map.size(); ++j)
        {
            edges_matrix[i][j] = 0.0;
        }
    }

    int nodeIndex = 0;

    nodesToPub.data.push_back(adj_map.size());

    for (it = adj_map.begin(); it!=adj_map.end(); it++)
    {

        float nodex = (it->first).y * resolution + origin[1];
        float nodey = (it->first).x * resolution + origin[0] ;
        nodesToPub.data.push_back(nodex);
        nodesToPub.data.push_back(nodey);
        
        nodeList[nodeIndex][0] = (it->first).y;
        nodeList[nodeIndex][1] = (it->first).x;
        nodeIndex++;
    }



    nodeIndex = 0;
    for (it = adj_map.begin(); it!=adj_map.end(); it++)
    {
        for (int j=0; j<(it->second).size(); j++){
            int x1 = (it->second)[j].y;
            int y1 = (it->second)[j].x;

            int k = getIndexOfNode(x1, y1, nodeList, adj_map.size());
            if (k >= 0){ 
                float dist = distance(nodeList[nodeIndex][0], nodeList[k][0], nodeList[nodeIndex][1], nodeList[k][1]);
                edges_matrix[nodeIndex][k] = resolution * dist;
            }
        }
        nodeIndex++;
    }

    // for (int i = 0; i < adj_map.size(); ++i)
    // {
    //     for (int j = 0; j < adj_map.size(); ++j)
    //     {
    //         std::cout << edges_matrix[i][j] << ' ';
    //     }
    //     std::cout << std::endl;
    // }


    for (int i=0; i<adj_map.size(); i++){
        for (int j=0; j<adj_map.size(); j++){
            nodesToPub.data.push_back(edges_matrix[i][j]);
        }
    }

    pub_node.publish(nodesToPub);

    waitKey(6000);


    }

    else{
        cout << robot_name << ": Do nothing" << endl;
    }

}

void pose_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    
    int arrSize = msg->layout.dim[0].size;

    double poeses[arrSize] ;
    for (int i = 0; i < arrSize; i++){
        poeses[i] = msg->data[i];
    }
    int curx = (poeses[arrSize-2] + 20 ) * 20;
    int cury = (poeses[arrSize-1] + 20 ) * 20;
    if (curx + range > 799) curx = 799 - range;
    if (cury + range > 799) cury = 799 - range;
    if (curx - range <   0) curx =   0 + range;
    if (cury - range <   0) cury =   0 + range;

    curpose[0] = curx;
    curpose[1] = cury;

}


int main(int argc, char **argv)
{
    gridmap = new uint8_t*[height];
    for (int i = 0; i < height; i++) {
        gridmap[i] = new uint8_t[width];
    }
    
    for (int i = 0; i < height; i++){
        for (int j = 0; j < width; j++) {
            premap[i][j] = 255;
            prepremap[i][j] = 255;
            preprepremap[i][j] = 255;
            prepreprepremap[i][j] = 255;
        }
    }
        
    

    ros::init(argc, argv, "graph_generator");
    ros::NodeHandle nh;
    ros::param::get("~robot_name", robot_name);


    pub_node = nh.advertise<std_msgs::Float32MultiArray>("nodes", 1);
    ros::Subscriber move_sub =  nh.subscribe<std_msgs::Bool>("move_status", 1, move_cb);
    ros::Subscriber graph_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, mapCallback);
    ros::Subscriber pose_sub =  nh.subscribe<std_msgs::Float32MultiArray>("nav_goal", 1,pose_cb);

    ros::spin();

    return 0;
}
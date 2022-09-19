#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib> 
#include <iostream>
#include <bits/stdc++.h>
#include <fstream>
#include <string> 
#include <vector>
#include <sstream>
#include <algorithm> 
#include <cmath> 
#include <math.h> 
#include "unistd.h"

using namespace cv;
using namespace std;
#define val 25

// Draw a single point
static void draw_point( Mat& img, Point2f fp )
{
    circle( img, fp, 3, Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA, 0 );
}

struct ComparePoints
{
    bool operator () (const cv::Point& a, const cv::Point& b) const
    { 
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    }
};

struct str{
    bool operator() ( Point a, Point b ){
        if ( a.x != b.x ) 
            return a.x < b.x;
        return a.y <= b.y ;
    }
} comp;

// Function to calculate distance
static float distance(int x1, int x2, int y1, int y2)
{
    // Calculating distance
    return sqrt(pow(x2 - x1, 2) +
                pow(y2 - y1, 2) * 1.0);
}

static Point2f find_closest( vector<Point2f> contours, int x_val, int y_val ) {
    int closest_x = INT_MAX; 
    int closest_y = INT_MAX; 
    int mindist = INT_MAX;
    // find closest based on min distance formula dist of closest_x-x_val and closest_y-y_val is lesser than that for contours[i].x and contours[i].y
    for (int i = 0; i < contours.size(); i++) {
        if ( closest_x == INT_MAX && closest_y == INT_MAX ) {
                closest_x = contours[i].x;
                closest_y = contours[i].y;              
        }
        else {
            if ( distance(contours[i].x, x_val, contours[i].y, y_val ) <= distance(x_val, closest_x, y_val, closest_y ) ) {
                closest_x = contours[i].x;
                closest_y = contours[i].y;              
            }
        }
    }

    // closest_x and closest_y contain point with closest magnitude to that of current point 
    return Point2f(closest_x, closest_y);
}

static vector<vector<string>> parse_csv( )
{
    vector<vector<string>> content;
    vector<string> row; 
	string line, word;
 
    // other csv: map_uint8.csv
	fstream file ("maplow.csv", ios::in);
	if(file.is_open())
	{
        size_t i = 0; 
        // size_t ct = 0; 
		while(getline(file, line))
		{
			stringstream str(line);
            row.clear(); 
            
            // DISCARD FIRST ROW

			while(getline(str, word, ',')) {
                // std::cout << word << std::endl; 
				row.push_back(word);
                // std::cout << "word: " << word << std::endl; 
            } 

            // std::cout << row << std::endl;

            // UNCOMMENT FOLLOWING LINES TO PRINT OUT PARSED CSV
            for ( size_t j = 0; j< row.size(); j++ ) {
                //  std::cout << "| " << row[j] << " "; 
            } 
            // std::cout << "stage 1" << std::endl;
            // std::cout << "[[[[[[[" << i << "]]]]]]] ";             
            i += 1; 
            // std::cout <<"pushing back row: " << i << std::endl;
            // std::cout << "[[[[[[[" << i << "]]]]]]] ";             


			content.push_back(row);

            // std::cout << "ct: [" << i << "] " << std::endl;

		}
        // std::cout << "stage 3" << std::endl;

	}
	// else
	// 	cout<<"Could not open the file\n";

    // std::cout << "stage 4" << std::endl;

    return content; 

	// for(int i=0;i<1500;i++)
	// {
	// 	for(int j=0;j<1500;j++)
	// 	{
	// 		data[i][j] = .push_back(content[i]);
	// 	}
	// 	cout<<"\n";
	// }    

}

static bool inside_rect( Rect rect, Point pt ) { 
    bool logic = ((rect.x < pt.x) && (pt.x < (rect.x+rect.width)) && (rect.y < pt.y) && ( pt.y < (rect.y+rect.height))); 

    // std::cout << rect.x << " " << pt.x << " " << rect.x+rect.width << std::endl;
    // std::cout << logic << std::endl;
    // std::cout << rect.y << " " << pt.y << " " << rect.y+rect.height << std::endl;  

    // if (logic == 0) { 
    //     std::cout << "FALSE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    // }
    return logic;
}

//Draw voronoi diagram
static void draw_voronoi( Mat& img, Subdiv2D& subdiv, Rect rect, float* avg_d, size_t* vert_ct, vector<Point2f> points, std::map<cv::Point, vector<Point>, ComparePoints>& adj_map, float node_dist )
{
    vector<vector<Point2f> > facets;
    vector<Point2f> centers;
    // std::cout << "test4: " << std::endl; 
    subdiv.getVoronoiFacetList(vector<int>(), facets, centers);
    // std::cout << "test5: " << std::endl; 

    vector<Point> ifacet;
    vector<vector<Point> > ifacets(1);

    unordered_map<string, int> freq_map;    


    Mat copy_im =img.clone(); 
    Mat voronoi_div = img.clone(); 

    vector<Point> facet_compare; 
    vector<Point> vertices;

    // std::map<cv::Point, vector<Point>, ComparePoints> adj_map;

    // std::cout << "test6: " << std::endl; 

    // freq_map[Point(2,3)] = 0;


    // 1. VORONOI PARTITION VERTICES -> GRAPH VERTICES (USING FREQUENCY OF OCCURRENCE- CREATING HISTOGRAM)

    for( size_t i = 0; i < facets.size(); i++ )
    {
        // std::cout << facets[i].size() << std::endl;
        ifacet.resize(facets[i].size());
        
        // print out each facets[i][j], see if there are repeats, if there are, need to turn them into voronoi vertices
        for( size_t j = 0; j < facets[i].size(); j++ ) {
            facet_compare.push_back(facets[i][j]);
            ifacet[j] = facets[i][j];
            // std::cout << "pot_vert: " << ifacet[j] << std::endl;
           
            string a = to_string(ifacet[j].x);
            string b = " "; 
            string c = to_string(ifacet[j].y);
            freq_map[a+b+c] += 1;
        }

        // std::cout << "ifacet: " << ifacet << std::endl;
        ifacets[0] = ifacet;

        // COMMENT OUT TO GET RID OF VORONOI DIV image generation
        polylines(voronoi_div, ifacets, true, Scalar(), 1, cv::LINE_AA, 0);
        // circle(img, centers[i], 3, Scalar(), cv::FILLED, cv::LINE_AA, 0);
    }
    // std::cout << "test7: " << std::endl; 
    
    // create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
    cv::Mat c_voronoi(img.size(), CV_8UC3);

    // convert grayscale to color image
    cv::cvtColor(voronoi_div, c_voronoi, COLOR_GRAY2BGR);        

    for( vector<Point2f>::iterator it = points.begin(); it != points.end(); it++)
    {
        draw_point(c_voronoi, *it);
    } 

    // 4. Show voronoi partitions based on boundary points
    // imshow( "voronoi partitions", c_voronoi);
    // imwrite("outputimgs/4.jpg", c_voronoi);

    // imshow( "copy_image", copy_im);


    // use completed freq hist to push back only points with freq > 2:  

    cv::Mat no_pts = img.clone();

    // OPTIMIZATION PROBLEM FOR LATER: FIX DOUBLE PLOTTING VORONOI VERTICES-- use frequency dictionary
    // plot voronoi vertices, by checking if a point is a member of two different voronoi facets:
    for (size_t i = 0; i < facet_compare.size(); i++ ) {
        if ( std::count (facet_compare.begin(), facet_compare.end(), facet_compare[i]) > 1 ) {

            // access output image, if pixel where facet_compare[i] is being plotted is black, DON'T PLOT THE PIXEL, DONT add to array!
            // can then get rid of map and vector 


            // READ FREQUENCY HISTOGRAM, PUSH POINTS ONTO GRAPH BASED ON FREQ

            // current version doesn't get rid of point generation in black regions 
            // UNCOMMENT (int) thing for vertex generation not on black regions-- DONT need if using Hough transform?? 
            if ( (freq_map[to_string(facet_compare[i].x) + " " + to_string(facet_compare[i].y)] > 2 ) && (int)(copy_im.at<uchar>(facet_compare[i])) != 0 && inside_rect(rect, facet_compare[i]) ) {// && (inside_rect(rect, facet_compare[i])))
                if ( vertices.size() >= 1 ) {
                    bool dont_add = false; 
                    // loop through other elements in vertices to see if within distance threshold
                    // don't add if there are other elements with dist threshold
             
                    // COMMENT OUT TO GET RID OF VORONOI TRIMMING: 
                    for ( size_t j = 0; j < vertices.size(); j++ ) {
                        // comment in circle mask to get obstacle trimming 
                        if ( distance(vertices[j].x, facet_compare[i].x, vertices[j].y, facet_compare[i].y) <= (node_dist) ){ //|| circle_mask(img, facet_compare[i], node_dist) ) {
                            dont_add = true; 
                        }
                    }

                    // loop through whole image to check boundary radius- WAY TOO INEFFICIENT 
                    // for(int k=0; k<img.rows; k++) {
                    //     for(int z=0; z<img.cols; z++) { 
                    //         if ( (pow((k-((int)facet_compare[i].x)),2) + pow((z-((int)facet_compare[i].y)),2) < pow(18,2) )) {
                    //             if ( img.at<uchar>(k,z) == 0 ) { 
                    //                 dont_add = true; 
                    //                 break; 
                    //             }                            
                    //         }

                    //     } 
                    // }
                    
                    if ( dont_add != true ) {
                    // ENSURE they don't go out of rectangle range (assuming top left of image is origin)
                    int x = ( facet_compare[i].x-(val/2) >= 0 ) ? facet_compare[i].x-(val/2) : 0;
                    int y = ( facet_compare[i].y-(val/2) >= 0 ) ? facet_compare[i].y-(val/2) : 0;
                    // int x = (facet_compare[i].x-9);
                    // int y = (facet_compare[i].y-9);

                    int width = val;
                    int height = val;

                    // our rectangle...
                    cv::Rect rad_rect(x, y, width, height);

                    rectangle(no_pts, rad_rect, 100, 3);

                    // std::cout << "X: " << x << " Y: " << y << std::endl;
                    for(int k=x; k<(x+width); k++) {
                    for(int z=y; z<(y+height); z++) { 
                    if ( (int)(no_pts.at<uchar>(Point(k,z))) == 0 ) {

                    dont_add = true; }}}
                    }

                    // 8. Show floorplan bounding rectangle
                    // imshow("rect", no_pts );   
                    // imwrite("outputimgs/rect.jpg", no_pts);                    



                    // if ( pow(facet_compare[i].x,2) + pow(facet_compare[i].x,2) )


                    if ( dont_add == false ) {
                        // std::cout << "dist: " << distance(vertices[vertices.size()-1].x, facet_compare[i].x, vertices[vertices.size()-1].y, facet_compare[i].y) << std::endl;                         
                        // COMMENT OUT TO GET RID OF VORONOI TRIMMING:                     
                        
                        // CHANGE CIRCLE COLOR TO SEE IF PLOTTING WITHIN BOUNDS ONLY IS WORKING!
                        circle(img, facet_compare[i], (node_dist/2), Scalar(0), 1, cv::LINE_AA, 0);                                
                        circle(img, facet_compare[i], 3, Scalar(100), cv::FILLED, cv::LINE_AA, 0);
                        freq_map[to_string(facet_compare[i].x) + " " + to_string(facet_compare[i].y)] -= 1;       
                        
                        // COMMENT OUT IF NOT GENERATING INTERMEDIATE PLOT ALL VORONOI VERTICES GRAPH 
                        circle(voronoi_div, facet_compare[i], (node_dist/2), Scalar(0), 1, cv::LINE_AA, 0);                                
                        // circle(voronoi_div, facet_compare[i], 3, Scalar(100), cv::FILLED, cv::LINE_AA, 0);

                        vertices.push_back(facet_compare[i]);
                        adj_map[facet_compare[i]];

                    }
                    else {
                        // std::cout << "NOT_ADDING, print vertex: " << facet_compare[i] << std::endl; 
                        // add to DONT_ADD LIST 

                        // COMMENT OUT IF NOT GENERATING INTERMEDIATE PLOT ALL VORONOI VERTICES GRAPH 
                        // circle(voronoi_div, facet_compare[i], (node_dist/2), Scalar(0), 1, cv::LINE_AA, 0);                                
                        circle(voronoi_div, facet_compare[i], 3, Scalar(100), cv::FILLED, cv::LINE_AA, 0);

                        // trimmed_vertices.push_back(facet_compare[i]); 
                    }
                }
                else {
                    // COMMENT OUT TO GET RID OF VORONOI TRIMMING: 
                    // circle(img, facet_compare[i], (node_dist/2), Scalar(0), 1, cv::LINE_AA, 0);                                
                    // circle(img, facet_compare[i], 3, Scalar(100), cv::FILLED, cv::LINE_AA, 0);
                    // freq_map[to_string(facet_compare[i].x) + " " + to_string(facet_compare[i].y)] -= 1;       
                    // vertices.push_back(facet_compare[i]);
                    // adj_map[facet_compare[i]];
                    bool dont_add = false; 

                    int x = ( facet_compare[i].x-(val/2) >= 0 ) ? facet_compare[i].x-(val/2) : 0;
                    int y = ( facet_compare[i].y-(val/2) >= 0 ) ? facet_compare[i].y-(val/2) : 0;

                    int width = val;
                    int height = val;

                    // our rectangle...
                    cv::Rect rad_rect(x, y, width, height);

                    rectangle(no_pts, rad_rect, 100, 3);


                    // std::cout << "X: " << x << " Y: " << y << std::endl;
                    // size_t new_fac = 0; 
                    for(int k=x; k<(x+width); k++) {
                        for(int z=y; z<(y+height); z++) { 
                            if ( (int)(no_pts.at<uchar>(Point(k,z))) == 0 ) {

                                // circle(no_pts, Point(k,z), 1, Scalar(0), cv::FILLED, cv::LINE_AA, 0);                                
                                dont_add = true;
                                // new_fac += 1;  
                            }
                        }
                    }

                    if ( dont_add == false ) {
                        circle(img, facet_compare[i], (node_dist/2), Scalar(0), 1, cv::LINE_AA, 0);                                
                        circle(img, facet_compare[i], 3, Scalar(100), cv::FILLED, cv::LINE_AA, 0);
                        freq_map[to_string(facet_compare[i].x) + " " + to_string(facet_compare[i].y)] -= 1;       
                        vertices.push_back(facet_compare[i]);
                        adj_map[facet_compare[i]];
                    }



                }
            }            
        }        
    }    

    // 5. Show map with all vertices plotted
    // imshow( "all vertices", voronoi_div);
    // imwrite("All_vertices.jpg", voronoi_div);
    // imwrite("outputimgs/5.jpg", voronoi_div);
    
    float dist_sum = 0; 
    int num_untrimmed_edges = 0;
    for ( size_t i = 0; i < vertices.size(); i++ ) {
        for ( size_t j = i+1; j < vertices.size(); j++ ) {
            // ADJACENCY PARAMETER = 120
            float dist_val = distance(vertices[i].x, vertices[j].x, vertices[i].y, vertices[j].y ); 
            // experiment with node_dist * 3 (factor of three for edge generation)
            if ( dist_val <= (node_dist * 3))  {
                (adj_map[vertices[i]]).push_back(vertices[j]); 
                dist_sum += dist_val; 
                num_untrimmed_edges += 1; 
                // std::cout << "(" << vertices[j].x << ", " << vertices[j].y << ")" << std::endl;  
                // std::cout << "vertices example sizes: s1 = " << adj_map[vertices[i]].size() << endl; 
            }
        }
    }

    float avg_dist = (float) dist_sum / (float) num_untrimmed_edges;  
    std::map<cv::Point, vector<Point>, ComparePoints>::iterator it1;
    
    float std_sum = 0;
    for (it1 = adj_map.begin(); it1 != adj_map.end(); it1++)
    {
        for ( size_t k = 0; k < (it1->second).size(); k++ ) {
            float dist_val = distance((it1->first).x, (it1->second)[k].x, (it1->first).y, (it1->second)[k].y );            
            std_sum += pow((dist_val-avg_dist),2);
        }
    }

    float stddev = sqrt( (float)std_sum/ (float) num_untrimmed_edges);

    // std::cout << "STDDEV: " << stddev << std::endl; 
    // std::cout << "STDDEV: " << stddev << std::endl; 
    // std::cout << "STDDEV: " << stddev << std::endl; 
    // std::cout << "STDDEV: " << stddev << std::endl; 
    // std::cout << "STDDEV: " << stddev << std::endl; 
    // std::cout << "STDDEV: " << stddev << std::endl; 
    // std::cout << "STDDEV: " << stddev << std::endl; 


    // 6. Plot trimmed vertices without generated edges
    // imshow( "vertices w/o edges", img);
    // imwrite("vertexnoedge.jpg", img);
    // imwrite("outputimgs/6.jpg", img); 


    Mat all_edges = img.clone(); 

    // Generate edges between points based on adjacency map
    std::map<cv::Point, vector<Point>, ComparePoints>::iterator it;

    size_t num_legal_lines = 0;
    
    for (it = adj_map.begin(); it != adj_map.end(); it++)
    {
        for ( size_t k = 0; k < (it->second).size(); k++ ) {
            LineIterator itr(img, it->first, (it->second)[k], 8);

            int add_line = 0; 

            if ( distance((it->first).x, (it->second)[k].x, (it->first).y, (it->second)[k].y ) > (avg_dist+stddev/2.0))
                add_line += 1; 
            
            // loop through the entire proposed line to see if any sections cross over black pixels
            for(int b = 0; b < itr.count; b++, ++itr) {
                Point pt= itr.pos();

                if ( (int)(copy_im.at<uchar>(pt) == 0 ) ) { 
                    add_line += 1;
                    // plot all of the points that were not added!
                    circle(img, pt, 3, Scalar(100), cv::FILLED, cv::LINE_AA, 0);        
                    break;  
                }

                // circle(img,pt,2,0);
            }

            if ( add_line < 1 ) { 
                line(img, it->first, (it->second)[k], Scalar(), 2, LINE_8);
                line(all_edges, it->first, (it->second)[k], Scalar(), 2, LINE_8);
                num_legal_lines += 1;
                
            } 
            else {
                line(all_edges, it->first, (it->second)[k], Scalar(), 2, LINE_8);
            }
        }
    }

    // std::cout << "NUMLEGALLINES: " << num_legal_lines << std::endl;

    // 7. Plot trimmed vertices with all edges 
    // imshow("edges_untrimmed", all_edges);    
    // imwrite("edges_untrimmed.jpg", all_edges);   
    // imwrite("outputimgs/7.jpg", all_edges);  
    
    // CHECK IF POINT from ifacet list IS ON DONT_ADD LIST, if it is, then dont plot it. 
    // plot polylines now:
    // 3. PLOT VORONOI PARTITIONS from IFACET as one long POLYLINE (connected sequence of points)
    for( size_t i = 0; i < facets.size(); i++ )
    {
        ifacet.resize(facets[i].size());

        for( size_t j = 0; j < facets[i].size(); j++ ) {
            ifacet[j] = facets[i][j];
        }

        // std::cout << "ifacet: " << ifacet << std::endl;
        ifacets[0] = ifacet;

        // potential improvement: delete polyline if it contains illegal vertex???
        // polylines(img, ifacets, true, Scalar(), 1, cv::LINE_AA, 0);

    }    

    *avg_d = avg_dist; 

    *vert_ct = vertices.size();     

}
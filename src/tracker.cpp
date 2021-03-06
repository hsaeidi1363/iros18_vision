#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<sensor_msgs/Image.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Twist.h>
#include "planner.h"
#include "Eigen/Dense"
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 

#define PI 3.14159 

using namespace cv;
using namespace std;
using namespace cv_bridge;


// raw input image
Mat img;
// pointer for reading ros images via cv brdige
CvImagePtr cv_ptr;

// maximum number of corners to be detected for the camera calibration using the reference points


// canny edge detection threshold
int thresh_low = 30;
int thresh_high = 220;


// for producing random colors in the detected features
RNG rng(12345);

// initialization check to see if the first image is received or not
bool initialized = false;

// max x and y coordinates take from the camera
double width, height;

// references calibration points for the camera (the points on the corners of input image which will be compared against the corners of gauze) 
vector<Point2f> image_corners;
	
// callback function for the input image
void get_img(const sensor_msgs::Image &  _data){
	// read the rectified rgb input image from the camera
	cv_ptr = toCvCopy(_data,"rgb8");
	// take the image part and put it in a mat variable in opencv format
	img = cv_ptr->image;
	// get the width and height
	width = _data.width;
	height = _data.height;
	
	if (!initialized){
		image_corners.push_back(Point2f(0.0f, height));
		image_corners.push_back(Point2f(width, height));
		image_corners.push_back(Point2f(0.0f,0.0f));
		image_corners.push_back(Point2f(width, 0.0f));
	}
	initialized = true;
}


geometry_msgs::Twist haptic_pos;

void get_haptic (const geometry_msgs::Twist & _data){
	haptic_pos = _data;
}

bool test_start;

void get_test_start(const std_msgs::Bool & _data){
	test_start = _data.data;
}


geometry_msgs::Twist rob_pos;

void get_pos (const geometry_msgs::Twist & _data){
	rob_pos = _data;
}


std_msgs::UInt8 control_mode;
void get_mode(const std_msgs::UInt8 & _data){
	control_mode = _data;
}

// find the distance between to XY points
double distance (double x1, double y1, double x2, double y2){
	double dx = x1-x2;
	double dy = y1-y2;
	return (sqrt(dx*dx+dy*dy));
}

int main(int argc, char * argv[]){
	ros::init(argc,argv,"tracker");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

	string H_file = "/homography/H.yml";
	H_file = ros::package::getPath("iros18_vision")+H_file;
        FileStorage fs(H_file.c_str(), FileStorage::READ);

	H_file = "/homography/H_inv.yml";
	H_file = ros::package::getPath("iros18_vision")+H_file;
        FileStorage fs_inv(H_file.c_str(), FileStorage::READ);
	
	Mat offline_H;
	Mat offline_H_inv;    
	fs["Homography"] >> offline_H;
	fs_inv["Homography"] >> offline_H_inv;

        vector<vector<Point> > contours;
	// contours inside the region of interest
	vector<vector<Point> > masked_contours;
	// variable used for contour detection
	vector<Vec4i> hierarchy;
        
        

	bool circle_detection = false;
	bool contour_detection = false;
	bool partial_trajectory = false;
	bool offline_homography = false;
        bool track_detected = false;
        
        int track_ctr = 0;
	// number of waypoints sent to the robot
	int npt = 1;
	int roi_l = 0;
	int roi_r = 0;
	int roi_u = 0;
	int roi_b = 0;
	home.getParam("circle", circle_detection);
	home.getParam("contour", contour_detection);	
	home.getParam("partial_traj", partial_trajectory);
	home.getParam("offline_homography", offline_homography);
	home.getParam("number_of_waypoints", npt);	
	home.getParam("roi_l", roi_l);	
	home.getParam("roi_r", roi_r);
	home.getParam("roi_u", roi_u);
	home.getParam("roi_b", roi_b);

	// set the loop frequency equal to the camera input
	int loop_freq = 7;
	ros::Rate loop_rate(loop_freq);

	// subscribe to the rectified rgb input (needs the calibration file read in the launch file for the camera)
	ros::Subscriber cam_sub = nh_.subscribe("camera/image_rect_color",2,get_img);
	
	// subscriber to the robot position
	ros::Subscriber rob_sub = nh_.subscribe("/robot/worldpos", 10, get_pos);
        
        // subscriber to the haptic stylus position
	ros::Subscriber haptic_sub = nh_.subscribe("/manual_commands", 10, get_haptic);
        
        //subscriber to the test start trigger
        ros::Subscriber test_start_sub = nh_.subscribe("/test_start", 10, get_test_start);
        

	// subscriber to the robot control mode
	ros::Subscriber control_mode_sub = nh_.subscribe("/iiwa/control_mode" , 10, get_mode);

	//publisher for checking the images and debugging them
	ros::Publisher dbg_pub = nh_.advertise<sensor_msgs::Image>("mydbg",1);
	// publisher for checking some numerical values
	ros::Publisher dbg2_pub = nh_.advertise<geometry_msgs::Twist>("imgdbg",1);
	// publisher for sending the point plans to the trajectory generator of the robot
	ros::Publisher plan_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/plan",1);



	// a debugging variable
	geometry_msgs::Twist x;
	// the world coordinates of the reference calibration square
	vector<Point2f> rect;
//	rect.push_back(Point2f(0.1f, -0.5f));
//	rect.push_back(Point2f(0.1f, -0.4f));
//	rect.push_back(Point2f(0.0f, -0.5f));
//	rect.push_back(Point2f(0.0f, -0.4f));

	rect.push_back(Point2f(0.0315f, -.59521f));
	rect.push_back(Point2f(0.03102f, -0.49823f));
	rect.push_back(Point2f(-0.0663f, -0.59548f));
	rect.push_back(Point2f(-0.06665f, -0.49814f));

	// variable for filtering the noise in the corner jumps
	vector<Point2f> prev_corners_sorted;	
	prev_corners_sorted.push_back(Point2f(0.0f, 0.0f));
	prev_corners_sorted.push_back(Point2f(0.0f, 0.0f));
	prev_corners_sorted.push_back(Point2f(0.0f, 0.0f));
	prev_corners_sorted.push_back(Point2f(0.0f, 0.0f));
	

	// the Homography transformation set to Identity matrix initially
	/*Eigen::MatrixXd H(3,3);
	Eigen::MatrixXd F(3,3);
	Eigen::MatrixXd FH(3,3);
	F << 1417.429951, 0.000000, 641.887498, 0.000000, 1416.676900, 456.966163, 0.000000, 0.000000, 1.000000;
	H << 1,0,0,0,1,0,0,0,1;
	FH <<1,0,0,0,1,0,0,0,1;
	*/	
	// two temp variables for transforming pixel coordinates to world frame coordinates
	Eigen::MatrixXd X(3,1);
	Eigen::MatrixXd Xp(3,1);

	// previous waypoints sent to the robot (used for filtering purposes)
	vector<Point2f> prev_way_points;
	// waypoints sent to the robot
	vector<Point2f> way_points;
	// initialize the previous waypoints
	for (int i = 0; i < npt; i++){
		Point pt_tmp;
		pt_tmp.x = 0; pt_tmp.y=0;		
		prev_way_points.push_back(pt_tmp);
	}
	cv::Rect roi;
        roi.x = roi_l;
        roi.y = roi_u;
        roi.width = roi_r - roi_l;
        roi.height = roi_b - roi_u;


	Point ul;
	Point br;
	ul.x = roi.x;
	ul.y = roi.y;
	br.x = roi.x + roi.width;
	br.y = roi.y + roi.height;

	while(ros::ok()){
		// if an image is received from the camera process it 
		if(initialized){
			image_corners[0].x = ul.x;
			image_corners[0].y = br.y;
			image_corners[1].x = br.x;
			image_corners[1].y = br.y;
			image_corners[2].x = ul.x;
			image_corners[2].y = ul.y;
			image_corners[3].x = br.x;
			image_corners[3].y = ul.y;


			// variable for grayscale format of the input image
			Mat gimg, hsv1, g0, g1;
			// convert the input image to grayscale
			cvtColor(img,gimg, CV_RGB2GRAY);
                        inRange(gimg,  Scalar(0), Scalar(40), g0);
                        cvtColor(img,hsv1, CV_RGB2HSV);
                        inRange(hsv1,  Scalar(0, 0, 0), Scalar(180, 255, 85), g1);
                        gimg = g0 | g1;
			// apply gaussian smoothing of the image
			GaussianBlur(gimg, gimg, Size(25,25),2,2);			
			// the corners of the reference square on the ground (pixel coordinates)

			
			Mat Hh;
			// find the homography transformation using the world frame coordinates for rect and their associate pixel frame coordinates
			
			//// beginning of the corner detection 			
                            vector<Point2f> corners;
                            // parameters of the corner detection algorithm
                            int maxCorners = 10;
                            double qualityLevel = 0.01;
                            double minDistance = 100;
                            int blockSize = 2;
                            bool useHarrisDetector = false;
                            double k = 0.04;
                            // detect the corners of the square using the smoothed grayscale image
                            goodFeaturesToTrack( gimg,
                                                corners,
                                                maxCorners,
                                                qualityLevel,
                                                minDistance,
                                                Mat(),
                                                blockSize,
                                                useHarrisDetector,
                                                k );

                            // sort the corners using the image corners to match order of the rect corners
                            vector<double> sort_tmp;
                            vector<Point2f> corners_sorted;			
                            // find the point closest to each corner of the image 
                            for (int i = 0 ; i < image_corners.size() ; i++){
                                    double min_dist = 10000;
                                    double min_ind = 0;
                                    for (int j = 0; j < corners.size(); j++){
                                            double dist = distance(image_corners[i].x,image_corners[i].y,corners[j].x,corners[j].y);
                                            if (dist < min_dist){
                                                    min_ind = j;
                                                    min_dist = dist;
                                            }
                                    }
                                    corners_sorted.push_back(corners[min_ind]);
                            }
                            //filter the position of the corners to prevent suddent jumps
                            double tau = 0.8;
                            for (int i = 0; i < corners_sorted.size() ; i++){
                                    corners_sorted[i].x = (1-tau)*corners_sorted[i].x + tau*prev_corners_sorted[i].x;
                                    corners_sorted[i].y = (1-tau)*corners_sorted[i].y + tau*prev_corners_sorted[i].y;
                                    prev_corners_sorted[i].x = corners_sorted[i].x;
                                    prev_corners_sorted[i].y = corners_sorted[i].y;
                            }
                            // show the corners
                            for( int i = 0; i < corners_sorted.size(); i++ ){ 
                                    //circle( img, corners_sorted[i], 6, Scalar(255,0,0), -1, 8, 0 );
                            }

                        if(!offline_homography){
				Hh = findHomography( corners_sorted, rect, CV_RANSAC );
			}else{
				Hh = offline_H;
			}
			
			vector<Point2f> dst;
                        // define the dimensions of the new view of the square (10x10 cm which is transformed to dim x dim pixels here
                        float dim = 500;
                        float pix2mm = 100/dim;
                       
                        dst.push_back(Point2f(1.0f, dim));
                        dst.push_back(Point2f(dim, dim));
                        dst.push_back(Point2f(1.0f, 1.0f));
                        dst.push_back(Point2f(dim, 1.0f));
                        Mat square_img = Mat::zeros( dim, dim, img.type() );
                        Mat square_overlay_img = Mat::zeros( dim, dim, img.type() );
                        Mat overlay_img = Mat::zeros(  height, width, img.type() );
                        //find the perspective transformation and extract the new frame
                        Mat scene2square, square2scene;
                        scene2square = getPerspectiveTransform(corners_sorted,dst);
                        square2scene = getPerspectiveTransform(dst, corners_sorted);
                        
                        warpPerspective(img,square_img,scene2square,square_img.size() );
                        
                        Mat blood;
                        Mat hsv;
                        cvtColor(square_img, hsv, COLOR_BGR2HSV);
                        
                        inRange(hsv,  Scalar(0, 100, 0), Scalar(160, 255, 100), blood);
                        GaussianBlur(blood, blood, Size(5,5),2,2);
                        blood = 255 - blood;
                        
                        SimpleBlobDetector::Params params_blood;
                        // Change thresholds
                        params_blood.minThreshold = 0;
                        params_blood.maxThreshold = 255;
                        
                        // Filter by Area.
                        params_blood.filterByArea = true;
                        params_blood.minArea = 500;

                      
                        
                        // Filter by Convexity
                        params_blood.filterByConvexity = true;
                        params_blood.minConvexity = 0.2;

                        // Filter by Inertia
                        params_blood.filterByInertia = true;
                        params_blood.minInertiaRatio = 0.1;


                        Ptr<SimpleBlobDetector> detector_blood = SimpleBlobDetector::create(params_blood);
                        
                        // Detect blobs.
                        vector<KeyPoint> keypoints_blood;
                        detector_blood->detect(blood, keypoints_blood);

                        
                        // Draw detected blobs as red circles.
                        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
                        Mat im_with_blood;
                        drawKeypoints( blood, keypoints_blood, im_with_blood, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

                        Point traj_center = Point(dim/2, dim/2); 
                        int traj_diag = (int)(dim*24.5/100);
                        Scalar traj_color = Scalar(0,0,255);
                        circle(im_with_blood, traj_center, traj_diag, traj_color, 1, LINE_AA);
                        
                        vector<vector<Point> > blood_overlaps;
                        for (int blood_no =0; blood_no < keypoints_blood.size(); blood_no++){
                            vector<Point> single_overlap;
                            float x_blood = keypoints_blood[blood_no].pt.x;
			    float y_blood = keypoints_blood[blood_no].pt.y;
			    float r_blood = keypoints_blood[blood_no].size/2;
                            int angles_no = 720;
                            double start_angle = 0;
                            double end_angle = 0;
                            for (int angles = 0; angles < angles_no ; angles ++){
                                float current_angle = (float) -angles/angles_no*PI*2;
                                float ptx = traj_center.x + traj_diag * cos(current_angle);
                                float pty = traj_center.y + traj_diag * sin(current_angle);
                                
                                float dist_to_blood = (ptx - x_blood)*(ptx - x_blood) + (pty - y_blood)*(pty - y_blood); 
                                // if this point of trajectory is inside the blood circle
                                if (dist_to_blood <= r_blood*r_blood){                                
                                    single_overlap.push_back(Point(ptx,pty));
                                    if(single_overlap.size() ==1){
                                        start_angle = current_angle;
                                    }
                                    circle( im_with_blood, Point(ptx,pty), 1, Scalar(255,0,0), -1, 8, 0 );
                                    end_angle = current_angle;
                                }
                                
                            }
                            int overllap_len = single_overlap.size();
                            if (overllap_len >2){
                                int mid_point_ind = (int) overllap_len / 2;
                                float mid_point_angle = (start_angle + end_angle)/2;
                                float start2end_angle = (start_angle - end_angle)/2; // be wary of the sign!
                                float go_back_angle = mid_point_angle + 1.61 * start2end_angle;
                                float go_forward_angle = mid_point_angle - 1.61 * start2end_angle;
                                double switch_start_x = traj_center.x + traj_diag * cos(go_back_angle);
                                double switch_start_y = traj_center.y + traj_diag * sin(go_back_angle);
                                double switch_end_x = traj_center.x + traj_diag * cos(go_forward_angle);
                                double switch_end_y = traj_center.y + traj_diag * sin(go_forward_angle);
                                Point rook_points[1][3];
                                rook_points[0][0] = Point( traj_center.x, traj_center.x );
                                //rook_points[0][1] = Point( single_overlap[overllap_len-1].x, single_overlap[overllap_len-1].y );
                                //rook_points[0][2] = Point( single_overlap[0].x, single_overlap[0].y );
                                rook_points[0][1] = Point( switch_start_x, switch_start_y );
                                rook_points[0][2] = Point( switch_end_x,switch_end_y);
                                const Point* ppt[1] = { rook_points[0] };
                                int npt[] = { 3 };
                                //fillPoly(  square_overlay_img, ppt, npt,1, Scalar( 0, 255, 0 ), 8 );
                                //circle( square_overlay_img, single_overlap[mid_point_ind], 10, Scalar(255,0,0), -1, 8, 0 );
                                
                            }
                            blood_overlaps.push_back(single_overlap);
                            
                        }
                        


			// for circle detection algorithm 
			if (circle_detection){
				vector<Vec3f> circles;
				// use the hough detection algorithm to find the circles			
				HoughCircles(gimg, circles, HOUGH_GRADIENT, 1, gimg.rows/8,200,50,0,0);
				for( size_t i = 0; i < circles.size();i++){
					Vec3i c = circles[i];
					circle(img, Point(c[0], c[1]) , c[2], Scalar(0,255,255), 3, LINE_AA);
					circle(img, Point(c[0], c[1]), 2, Scalar(0,255,255), 3, LINE_AA);
					vector< vector<double> > points = get_points(c[0], c[1], c[2], npt);
					for (int j = 0; j < npt; j++){
						circle(img, Point(points[j][0], points[j][1]), 5, Scalar(255,0,0), 3, LINE_AA);
					} 
				}
			}
			///

		
	
			
			
			
			///
			if (contour_detection){
				Mat canny_output;
			
                                if (!track_detected || track_ctr <20){
                                    track_ctr++;
                                    track_detected = true;
                                    contours.clear();
                                    hierarchy.clear();
                                  
                                    /// Detect edges using canny
                                    Canny( gimg, canny_output, thresh_low, thresh_high, 3 );
                                    gimg = canny_output;
                                    /// Find contours
                                    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );
                                }
                                masked_contours.clear();
                                /// detect the contours that are close to the center of image
                                for( int i = 0; i< contours.size(); i++ ){
                                        bool in_region =true;
                                        for (int k = 0; k < contours[i].size(); k++){
                                                Point pt = contours[i][k];
                                                if (distance(pt.x, pt.y, width/2,height/2) > 300){
                                                        in_region = false;
                                                }
                                        }
                                        if(in_region){	
                                                // collect the contours in the region of interest
                                                masked_contours.push_back(contours[i]);			
                                                //drawContours( img, contours, i, Scalar(255,255,0), 2, 8, hierarchy, 0, Point() );
                                                // only draw the first one for now		
                                                if(masked_contours.size() == 1){
                                                        drawContours( img, contours, i, Scalar(255,255,0), 2, 8, hierarchy, 0, Point() );

                                                        // calculate the length of the contour in pixels (from the start pixel to the end pixel)
                                                        int cont_size = masked_contours[0].size();
                                                        // add the distance between the final and initial point as a starting value (when sweeping the points the all of the points except the last are considered)
                                                        double cont_len = distance(masked_contours[0][0].x,masked_contours[0][0].y, masked_contours[0][cont_size-1].x,masked_contours[0][cont_size-1].y);
                                                        // calculate the length of the contour in pixels
                                                        for (int k = 0; k < cont_size-1; k++){
                                                                double ds = distance(masked_contours[0][k].x,masked_contours[0][k].y, masked_contours[0][k+1].x,masked_contours[0][k+1].y); 
                                                                cont_len += ds;
                                                        }
                                                
                                                        Point pt;
                                                        double seg_len = 0;
                                                        double seg_len_prev = 0;
                                                        way_points.clear();	
                                                        if(partial_trajectory){
                                                                double min_dist = 10000;
                                                                int min_ind = 0;
                                                                vector<Point2f> contour_world;
                                                                vector<Point2f> contour_inp;
                                                                for (int kk = 0; kk<masked_contours[0].size(); kk++)
                                                                        contour_inp.push_back(masked_contours[0][kk]);
                                                                perspectiveTransform(contour_inp, contour_world, Hh);
                                                                
                                                                for (int k = 0; k < cont_size; k++){
                                                                        double dist = distance(contour_world[k].x,contour_world[k].y, rob_pos.linear.x,rob_pos.linear.y); 
                                                                        if(dist < min_dist){
                                                                                min_dist = dist;
                                                                                min_ind = k;		
                                                                        }									
                                                                }
                                                                // pick the closest point as the first waypoint
                                                                pt = masked_contours[0][min_ind];
                                                                // put the first point of the contour in the waypoint list
                                                                way_points.push_back(pt);
                                                                
                                                                for (int k = 0; k < cont_size; k++){

                                                                        // calculate the length of a segment on the contour
                                                                        seg_len += distance(masked_contours[0][(min_ind+k)%cont_size].x,masked_contours[0][(min_ind+k)%cont_size].y, masked_contours[0][(min_ind+k+1)%cont_size].x,masked_contours[0][(min_ind+k+1)%cont_size].y); 
                                                                        // check if it is close to the average length (total_length/number_of_waypoints)
                                                                        if( (seg_len_prev < cont_len/npt) && (seg_len > cont_len/npt) ){
                                                                                // add this point to the waypoint list
                                                                                pt = masked_contours[0][(min_ind+k)%cont_size];
                                                                                way_points.push_back(pt);				
                                                                                // reset the length			
                                                                                seg_len = 0;
                                                                        }
                                                                        seg_len_prev = seg_len;
                                                                        if(way_points.size() == 3)
                                                                                break;
                                                                }	


                                                        }else{
                                                                pt = masked_contours[0][0];
                                                                // put the first point of the contour in the waypoint list
                                                                way_points.push_back(pt);
                                                                // scan through the points and try to produce equally distanced waypoints using the total length of the contour 
                                                                for (int k = 0; k < cont_size-1; k++){
                                                                        // calculate the length of a segment on the contour
                                                                        seg_len += distance(masked_contours[0][k].x,masked_contours[0][k].y, masked_contours[0][k+1].x,masked_contours[0][k+1].y); 
                                                                        // check if it is close to the average length (total_length/number_of_waypoints)
                                                                        if( (seg_len_prev < cont_len/npt) && (seg_len > cont_len/npt) ){
                                                                                // add this point to the waypoint list
                                                                                pt = masked_contours[0][k];
                                                                                way_points.push_back(pt);				
                                                                                // reset the length			
                                                                                seg_len = 0;
                                                                        }
                                                                        seg_len_prev = seg_len;
                                                                }	
                                                                // add the final point of the contour to the waypoint list
                                                                //pt = masked_contours[0][masked_contours[0].size()-1];
                                                                //way_points.push_back(pt);
                                                                // lowpass filter for the position of the way points, set tau = 0.0 to deactivate the filter
                                                                
                                                        }
                                                        double tau = 0.8;
                                                        // filter the noise in the waypoints (prevent suddent jumps)
                                                        for (int k = 0; k < way_points.size(); k++){
                                                                way_points[k].x = (1-tau)*way_points[k].x + tau*prev_way_points[k].x;
                                                                way_points[k].y = (1-tau)*way_points[k].y + tau*prev_way_points[k].y;
                                                        }
                                                        // save the last value for the next filtering loop
                                                        prev_way_points = way_points;
                                                }
                                        }
                                   
                            }//end of if
			// end of contour detection
			 }
			
/*
			image_corners.push_back(Point2f(width, height));
			image_corners.push_back(Point2f(0.0f,0.0f));
			image_corners.push_back(Point2f(width, 0.0f));
*/
			

			//for (int i = 0; i < 3; i++)
			//	for (int j = 0; j < 3; j++)
					//H(i,j) = Hh.at<double>(i,j);
		
/*
			Eigen::MatrixXd R(3,3);
			Eigen::MatrixXd T(1,3);
			R<< 0.999886810965460,    -0.0150454397517300,    0,0.0150454397517300,    0.999886810965460,    0, 0,    0,    1;			
			T << -0.0169027700559601,    -0.580519109823255,    -0.290000000000000;
*/
        
			std::vector<Point2f> scene_corners(4);
			std::vector<Point2f> scene_wps(way_points.size());

 			//perspectiveTransform( corners_sorted, scene_corners, Hh);
 			perspectiveTransform( way_points, scene_wps, Hh);
			trajectory_msgs::JointTrajectory plan;
			// a variable for changing the color of waypoints from red (first waypoint) to the green (last waypoint)
			int col_inc = 255/npt;

			for (int k = 0; k <way_points.size(); k++){
				trajectory_msgs::JointTrajectoryPoint plan_pt;
				plan_pt.positions.push_back(scene_wps[k].x);
				plan_pt.positions.push_back(scene_wps[k].y);
				double z_start = 0.73;
                                double z_cut = 0.672;
                                if(test_start)
                                    plan_pt.positions.push_back(z_cut);
                                else
                                    plan_pt.positions.push_back(z_start);
				plan_pt.positions.push_back(3.1);
				plan_pt.positions.push_back(0.002);
				plan_pt.positions.push_back(1.8415);
				for (int ind= 0; ind < 6; ind++){
					plan_pt.velocities.push_back(0);
					plan_pt.accelerations.push_back(0);
				}
				plan.points.push_back(plan_pt);			
				//circle(img, Point(way_points[k].x, way_points[k].y), 5, Scalar(255-k*col_inc,0+k*col_inc,0+k*col_inc), 3, LINE_AA);

			}
	
	

			plan_pub.publish(plan);
			// show the image with detected points
//			rectangle(img, ul, br,Scalar(255,0,0), 3, LINE_AA);

			if(offline_homography){
				//std::vector<Point2f> robot_tool;
                                //robot_tool.push_back(Point2f(rob_pos.linear.x,rob_pos.linear.y));    
                                //std::vector<Point2f> robot_tool_projection;
	 			//perspectiveTransform( robot_tool,robot_tool_projection, offline_H_inv);
				//line(img,Point(robot_tool_projection[0].x ,robot_tool_projection[0].y + 5),Point(robot_tool_projection[0].x ,robot_tool_projection[0].y - 5), Scalar(0,255,0), 0.5, LINE_AA);
                                //line(img,Point(robot_tool_projection[0].x + 5,robot_tool_projection[0].y ),Point(robot_tool_projection[0].x - 5,robot_tool_projection[0].y ), Scalar(0,255,0), 0.5, LINE_AA);
                                if (control_mode.data == 0){
                                    std::vector<Point2f> haptic_point;
                                    haptic_point.push_back(Point2f(haptic_pos.linear.x,haptic_pos.linear.y));
                                    std::vector<Point2f> haptic_point_projection;
                                    perspectiveTransform( haptic_point,haptic_point_projection, offline_H_inv);                          
                                    line(img,Point(haptic_point_projection[0].x ,haptic_point_projection[0].y + 5),Point(haptic_point_projection[0].x ,haptic_point_projection[0].y - 5), Scalar(0,255,0), 0.5, LINE_AA);
                                    line(img,Point(haptic_point_projection[0].x + 5,haptic_point_projection[0].y ),Point(haptic_point_projection[0].x - 5,haptic_point_projection[0].y ), Scalar(0,255,0), 0.5, LINE_AA);
                                }
			}
                        warpPerspective(square_overlay_img,overlay_img,square2scene,overlay_img.size() );
                        addWeighted( img, 1.0, overlay_img, 0.5, 0.0, img);
			Mat img_crop = img(roi);
			std::string control_text;
			if(control_mode.data == 1)
				control_text ="Current Mode: Manual";
			if(control_mode.data == 0)
				control_text ="Current Mode: Autonomous";
			putText(img_crop , control_text, Point(200,150), CV_FONT_HERSHEY_DUPLEX, 2, Scalar (255,255,255,255)); 
			
                       // cvtColor(gimg, img_crop, CV_GRAY2RGB);

                        cv_ptr->image = img_crop;
                        //cv_ptr->image = im_with_blood;
			dbg_pub.publish(cv_ptr->toImageMsg());
			
		}		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
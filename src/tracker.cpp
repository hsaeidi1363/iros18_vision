#include<ros/ros.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Twist.h>
#include "planner.h"
#include "Eigen/Dense"



using namespace cv;
using namespace std;
using namespace cv_bridge;



Mat img;
CvImagePtr cv_ptr;

int maxCorners = 4;

RNG rng(12345);

bool initialized = false;

double width, height;

vector<Point2f> refs;
	
void get_img(const sensor_msgs::Image &  _data){
	cv_ptr = toCvCopy(_data,"rgb8");
	img = cv_ptr->image;
	width = _data.width;
	height = _data.height;
	if (!initialized){
		refs.push_back(Point2f(0.0f, height));
		refs.push_back(Point2f(width, height));
		refs.push_back(Point2f(0.0f,0.0f));
		refs.push_back(Point2f(width, 0.0f));
	}
	initialized = true;
}

double distance (double x1, double y1, double x2, double y2){
	double dx = x1-x2;
	double dy = y1-y2;
	return (sqrt(dx*dx+dy*dy));
}


int main(int argc, char * argv[]){
	ros::init(argc,argv,"tracker");
	ros::NodeHandle nh_;
	int loop_freq = 7;
	ros::Rate loop_rate(loop_freq);
	ros::Subscriber cam_sub = nh_.subscribe("camera/image_rect_color",10,get_img);

	ros::Publisher dbg_pub = nh_.advertise<sensor_msgs::Image>("mydbg",1);
	ros::Publisher dbg2_pub = nh_.advertise<geometry_msgs::Twist>("imgdbg",1);
	geometry_msgs::Twist x;
	vector<Point2f> rect;
	rect.push_back(Point2f(0.0f, 0.f));
	rect.push_back(Point2f(0.144f, 0.f));
	rect.push_back(Point2f(0.0f, 0.144f));
	rect.push_back(Point2f(0.144f, 0.144f));
	
	Eigen::MatrixXd H(3,3);
	H << 1,0,0,0,1,0,0,0,1;
	Eigen::VectorXd X(3);
	Eigen::VectorXd Xp(3);


	while(ros::ok()){
		if(initialized){
			Mat cimg;
			
			
			cvtColor(img,cimg, CV_RGB2GRAY);
			GaussianBlur(cimg, cimg, Size(5,5),2,2);			
			vector<Point2f> corners;
			double qualityLevel = 0.01;
			double minDistance = 100;
			int blockSize = 10;
			bool useHarrisDetector = false;
			double k = 0.04;

			goodFeaturesToTrack( cimg,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );



			vector<Vec3f> circles;
			HoughCircles(cimg, circles, HOUGH_GRADIENT, 1, cimg.rows/8,200,50,0,0);
			for( size_t i = 0; i < circles.size();i++){
				Vec3i c = circles[i];
				circle(img, Point(c[0], c[1]) , c[2], Scalar(0,255,255), 3, LINE_AA);
				circle(img, Point(c[0], c[1]), 2, Scalar(0,255,255), 3, LINE_AA);
				int n_points = 1;
				vector< vector<double> > points = get_points(c[0], c[1], c[2], n_points);
				for (int j = 0; j < n_points; ++j){
					circle(img, Point(points[j][0], points[j][1]), 5, Scalar(255,0,0), 3, LINE_AA);
				} 
				Xp(0) = points[0][0]; Xp(1) = points[0][1]; Xp(2) = 1;
			}
			
			int r = 6;
			vector<double> sort_tmp;
			for( int i = 0; i < corners.size(); i++ )
			{ 
				circle( img, corners[i], r, Scalar(255,0,0), -1, 8, 0 );
			}

			vector<Point2f> corners_sorted;			
			for (int i = 0 ; i < refs.size() ; i++){
				double min_dist = 10000;
				double min_ind = 0;
				for (int j = 0; j < corners.size(); j++){
					double dist = distance(refs[i].x,refs[i].y,corners[j].x,corners[j].y);
					if (dist < min_dist){
						min_ind = j;
						min_dist = dist;
					}
				}
				corners_sorted.push_back(corners[min_ind]);
			}
				
			Mat Hh = findHomography( rect, corners_sorted, CV_RANSAC );


			cv_ptr->image = img;
			dbg_pub.publish(cv_ptr->toImageMsg());	
			

			H(0,0) = Hh.at<double>(0,0);
			H(0,1) = Hh.at<double>(0,1);
			H(0,2) = Hh.at<double>(0,2);
			H(1,0) = Hh.at<double>(1,0);
			H(1,1) = Hh.at<double>(1,1);
			H(1,2) = Hh.at<double>(1,2);
			H(2,0) = Hh.at<double>(2,0);
			H(2,1) = Hh.at<double>(2,1);
			H(2,2) = Hh.at<double>(2,2);



			X = H.inverse()*Xp;
			x.linear.x= Xp(0);
			x.linear.y= Xp(1);
			x.linear.z= Xp(2);
			x.angular.x= X(0);
			x.angular.y= X(1);
			x.angular.z= X(2);


			dbg2_pub.publish(x);
		}		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

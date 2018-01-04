#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>



using namespace cv;
using namespace std;
using namespace cv_bridge;

Mat img;

CvImagePtr cv_ptr;

bool initialized = false;

int width , height;

void get_img(const sensor_msgs::Image & _data){
	//read the rectified rgb input image from camera
	cv_ptr = toCvCopy(_data,"rgb8");
	// take the image component and put it in a Mat opencv variable
	img = cv_ptr->image;
	// get the width and height
	width = _data.width;
	height = _data.height;
	
	initialized = true;
}

double distance (double x1, double y1, double x2, double y2){
	double dx = x1 - x2;
	double dy = y1 - y2;
	return sqrt(dx*dx+dy*dy);

}
int main(int argc, char * argv[]){
	ros::init(argc, argv, "homography_est");
	ros::NodeHandle nh_;

	int roi_l = 0;
	int roi_r = 0;
	int roi_u = 0;
	int roi_b = 0;	

	ros::NodeHandle home("~");
	home.getParam("roi_l", roi_l);	
	home.getParam("roi_r", roi_r);	
	home.getParam("roi_u", roi_u);	
	home.getParam("roi_b", roi_b);	

	string file_name = "/homography/H.yml";
	file_name = ros::package::getPath("iros18_vision")+file_name; 

	FileStorage fs(file_name.c_str(), FileStorage::WRITE);

	int loop_freq = 7;
	ros::Rate loop_rate(loop_freq);
	
	ros::Publisher img_pub = nh_.advertise<sensor_msgs::Image>("homography_dbg", 1);

	ros::Subscriber cam_sub = nh_.subscribe("camera/image_rect_color", 10,get_img);
	
	vector<Point2f> rect;
	rect.push_back(Point2f(0.044f, -0.566f));
	rect.push_back(Point2f(0.0436f, -0.469f));
	rect.push_back(Point2f(-0.05345f, -0.5674f));
	rect.push_back(Point2f(-0.05393f, -0.470f));
	
	vector<Point2f> prev_corners_sorted;
	for (int i = 0; i < 4; i++)
		prev_corners_sorted.push_back(Point2f(0.0f,0.0f));
	
	Rect roi;
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

	vector<Point2f> image_corners;
	image_corners.push_back(Point2f(ul.x,br.y));		
	image_corners.push_back(Point2f(br.x,br.y));		
	image_corners.push_back(Point2f(ul.x,ul.y));		
	image_corners.push_back(Point2f(br.x,ul.y));	
	Mat Hh;// = (Mat_<double>(2,2) <<2.0,1.0,3.0,1.0);
	while(ros::ok()){
		if(initialized){
			Mat gimg;
			cvtColor(img, gimg, CV_RGB2GRAY);
			GaussianBlur(gimg, gimg, Size(5,5),2,2);
			vector<Point2f> corners;
			int maxCorners = 10;
			double qualityLevel = 0.01;
			double minDistance = 100;
			int blockSize = 2;
			bool useHarrisDetector = false;
			double k = 0.04;
			goodFeaturesToTrack( gimg, 
				corners,
				maxCorners,
				qualityLevel,
				minDistance,
				Mat(),
				blockSize,
				useHarrisDetector,
				k );


			vector<Point2f> corners_sorted; 
			for (int i = 0; i < image_corners.size(); i++){
				double min_dist = 10000;
				double min_ind = 0;
				for ( int j = 0; j < corners.size(); j++){
					double dist = distance(image_corners[i].x, image_corners[i].y, corners[j].x, corners[j].y);
					if(dist < min_dist){
						min_ind = j;
						min_dist = dist;
					}
				}
				corners_sorted.push_back(corners[min_ind]);

			}
			double tau = 0.8;
			for (int i = 0; i < corners_sorted.size(); i++){
				corners_sorted[i].x = (1-tau)*corners_sorted[i].x + tau*prev_corners_sorted[i].x;
				corners_sorted[i].y = (1-tau)*corners_sorted[i].y + tau*prev_corners_sorted[i].y;
				prev_corners_sorted[i].x = corners_sorted[i].x;
				prev_corners_sorted[i].y = corners_sorted[i].y;
				circle(img, corners_sorted[i],6,  Scalar(255,0,0), -1, 8,0);
			}
			Hh = findHomography(corners_sorted, rect, CV_RANSAC);

			Mat img_crop = img(roi);
			cv_ptr->image = img_crop;
			img_pub.publish(cv_ptr->toImageMsg());
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	fs << "Homography"<<Hh;
//	fs << Hh;
	fs.release();
	return 0;
}

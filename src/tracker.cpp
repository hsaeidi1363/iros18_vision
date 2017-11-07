#include<ros/ros.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;

Mat img;
CvImagePtr cvimage;
	
void get_img(const sensor_msgs::Image &  _data){
	cvimage = toCvCopy(_data);
	img = cvimage->image;
}

int main(int argc, char * argv[]){
	ros::init(argc,argv,"tracker");
	ros::NodeHandle nh_;
	ros::Subscriber cam_sub = nh_.subscribe("camera/image_raw",10,get_img);
	while(ros::ok()){
		ros::spinOnce();
	}
	return 0;
}

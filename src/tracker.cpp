#include<ros/ros.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Twist.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;

Mat img;
CvImagePtr cv_ptr;


bool initialized = false;
	
void get_img(const sensor_msgs::Image &  _data){
	cv_ptr = toCvCopy(_data,"rgb8");
	img = cv_ptr->image;
	initialized = true;
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
	while(ros::ok()){
		if(initialized){
			Mat cimg;
			
			cvtColor(img,cimg, CV_RGB2GRAY);
			GaussianBlur(cimg, cimg, Size(5,5),2,2);
			vector<Vec3f> circles;
			HoughCircles(cimg, circles, HOUGH_GRADIENT, 1, cimg.rows/8,200,50,0,0);
			for( size_t i = 0; i < circles.size();i++){
				Vec3i c = circles[i];
				circle(img, Point(c[0], c[1]) , c[2], Scalar(0,255,255), 3, LINE_AA);
				circle(img, Point(c[0], c[1]), 2, Scalar(0,255,255), 3, LINE_AA);
			}
//			cvtColor(cimg,img, CV_GRAY2RGB);//not sure about this
			cv_ptr->image = img;
			dbg_pub.publish(cv_ptr->toImageMsg());	
			x.linear.x = cimg.channels();
			x.linear.y = circles.size();
			dbg2_pub.publish(x);
		}		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

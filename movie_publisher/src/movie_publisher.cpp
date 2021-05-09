#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
	ros::init (argc, argv, "img_publisher");
	ros::NodeHandle nh("~"); 
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("image", 1);
	cv::Mat image;
	cv::VideoCapture camera;
	camera.open("/home/nuc01/Downloads/GOPRO260.MP4");
	if(!camera.isOpened()){
		ROS_INFO("failed to open camera.");
		return -1;
	}
	ros::Rate looprate (60); // capture image at 60Hz
	while(ros::ok()) {
		camera >> image;
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		image_pub.publish(msg);
		ros::spinOnce();
		looprate.sleep();
	}
	return 0;
}

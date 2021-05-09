#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "MVGE134GCT.hpp"



int main(int argc, char** argv) {
	ros::init (argc, argv, "img_publisher");
	ros::NodeHandle nh("~"); 
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("image", 10);
	cv::Mat image;
	CameraDevice camera;
	ros::Rate looprate (60); // capture image at 60Hz
	while(ros::ok()) {
		camera.PopFrame(image);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		image_pub.publish(msg);
		ros::spinOnce();
		looprate.sleep();
	}
	return 0;
}

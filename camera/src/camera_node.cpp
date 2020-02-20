#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>

static std::string gstreamer_pipeline (const size_t capture_width, const size_t capture_height, const size_t framerate, const int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

class CaptureDevice {

	const size_t capture_width_;
	const size_t capture_height_;
	const size_t framerate_;
	const size_t flip_method_;
	
	cv::VideoCapture capture_device_;

	public:
		CaptureDevice(const size_t capture_width, 
					  const size_t capture_height,
					  const size_t framerate, 
					  const int flip_method) :
 				capture_width_(capture_width), 
				capture_height_(capture_height),
				framerate_(framerate),
				flip_method_(flip_method)	{
			init();
		}
	
		~CaptureDevice() {
			release();
		}

		bool init() {
			std::string pipeline = gstreamer_pipeline(capture_width_,
												      capture_height_,
													  framerate_,
													  flip_method_);
	
			capture_device_ = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

			if (!capture_device_.isOpened()) {
				std::cout << "Failed to open capture device\n";	
				return false;
			}

			return true;
		}	

		void release() {
			capture_device_.release();
		}

		bool getImage(cv::Mat &image) {
			
			if (!capture_device_.read(image)) {
				std::cout << "Capture read error\n";
				return false;
			}

			return true;
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "camera_node");
	ros::NodeHandle nh;	

    image_transport::ImageTransport it(nh);
    image_transport::Publisher camera_image_publisher = it.advertise("/camera/image", 1);
    
	//ros::Publisher camera_image_publisher;
	//camera_image_publisher = nh.advertise<sensor_msgs::Image>("camera/image", 1);
	
	const size_t capture_width = 3840;
	const size_t capture_height = 2160;
	const size_t framerate = 10;
	const int flip_method = 0;

	std::string pipeline = gstreamer_pipeline(capture_width,
											  capture_height,
											  framerate,
											  flip_method);
	
	CaptureDevice capture_device(capture_width, capture_height, framerate, flip_method);

	cv::Mat image;
	ros::Rate rate(1);

	while (ros::ok()) {
		
		if (camera_image_publisher.getNumSubscribers() > 0) {	
		
			if (!capture_device.getImage(image)) {
				break;
			}
		    
		    cv::resize(image, image, cv::Size(640, 480));
			sensor_msgs::ImagePtr image_ptr_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			camera_image_publisher.publish(image_ptr_msg);
		} else {
			rate.sleep();
		}
		
		ros::spinOnce();
	}
}

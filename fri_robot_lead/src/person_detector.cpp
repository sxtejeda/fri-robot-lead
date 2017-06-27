#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "fri_robot_lead/PersonPresent.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

static const std::string FRONTAL_FACE_CASCADE = "/res/haarcascade_frontalface_alt2.xml";
static const std::string PROFILE_FACE_CASCADE = "/res/haarcascade_profileface.xml";

class ImageConverter{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher person_pub_;
	CascadeClassifier face_cascade_;
	CascadeClassifier profile_cascade_;
	std::string face_cascade_name_;
	std::string profile_cascade_name_;

public:
	ImageConverter()
		: it_(nh_) {
		
		std::stringstream face_stream;
		std::stringstream profile_stream;
		std::string ros_path = ros::package::getPath("fri_robot_lead");

		face_stream << ros_path << FRONTAL_FACE_CASCADE;
		profile_stream << ros_path << PROFILE_FACE_CASCADE;
		
		image_sub_ = it_.subscribe("/usb_cam/image_raw",1, &ImageConverter::imageCb, this);
		person_pub_ = nh_.advertise<fri_robot_lead::PersonPresent>("/person_present",10);
		if(!face_cascade_.load(face_stream.str()))
			ROS_ERROR_STREAM("PersonDetector: Failed to load face cascade from " << face_stream.str());
		if(!profile_cascade_.load(profile_stream.str()))
			ROS_ERROR_STREAM("PersonDetector: Failed to load profile cascade from " << profile_stream.str());

		namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter(){
		destroyWindow(OPENCV_WINDOW);	
	}

	bool personDetected(cv::Mat &frame){
		//setup a cv::Mat for usage in classifying
		std::vector<Rect> faces;
		Mat frame_gray;
		cvtColor(frame, frame_gray, CV_BGR2GRAY);
		equalizeHist(frame_gray, frame_gray);

		face_cascade_.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE,Size(30,30));

		if(faces.size() == 0){
			profile_cascade_.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE,Size(30,30));
		}

		return faces.size() != 0;
	}

	void imageCb(const sensor_msgs::ImageConstPtr &msg){
		cv_bridge::CvImagePtr cv_ptr;
		fri_robot_lead::PersonPresent pub_msg;
		Mat frame;

		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception &e){
			ROS_ERROR("Person Detector: cv_bridge exception: %s",e.what());
			return;
		}

		frame = cv_ptr->image;
		pub_msg.timeStamp = ros::Time::now();
		pub_msg.personPresent = personDetected(frame);
		person_pub_.publish(pub_msg);

		ROS_DEBUG("Image found");
		imshow(OPENCV_WINDOW,frame);
		waitKey(3);	
	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "person_detector");
	ImageConverter ic;
	ros::spin();
	return 0;

}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/**
* Receives a distorted image from ROS, undistort and publish back to ROS the
* rectified image;
*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // Image
    cv::Mat img;
    img = cv_bridge::toCvShare(msg, "bgr8")->image ;
    // Show image for debugging..
    cv::imshow("view", img);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  // Inialize ros within this node
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // Publisher and subscriber
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
  // Opencv visualizer for debug
  cv::namedWindow("view");
  cv::startWindowThread();

  ros::spin();

  cv::destroyWindow("view");
}

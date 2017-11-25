#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

image_transport::Publisher pub;

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
    // Camera parameters
    cv::Mat K,D;
    cv::Mat map1, map2;
    K = (cv::Mat_<double>(3,3) << 223.9244035941265, 0.0, 373.13793646453337, 0.0, 203.9183219803824, 243.39853351533412, 0.0, 0.0, 1.0);
    D = (cv::Mat_<double>(4,1) << 0.5559602724351955, 0.1651585718875876, -0.5417438146769518, 0.23737085106125386);
    cv::Size img_size(640,480);
    // Rectified map
    cv::fisheye::initUndistortRectifyMap( K,
                                          D,
                                          cv::Mat(),
                                          K,
                                          img_size,
                                          CV_16SC2,
                                          map1,
                                          map2 );
    // Rectifying image
    cv::remap(img, img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT,0);
    // Publish rectified image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub.publish(msg);
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
  pub = it.advertise("image_rect", 1);

  ros::spin();

}

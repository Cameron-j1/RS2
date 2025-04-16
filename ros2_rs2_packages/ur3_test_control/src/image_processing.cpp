#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber() : Node("image_subscriber")
  {
    // Create the subscription to the camera topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", 10,
      std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Image subscriber node initialized");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received image");
    
    try {
      // Convert ROS Image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      
      // Now cv_ptr->image is the cv::Mat object
      cv::Mat image = cv_ptr->image;
      
      // Example: Display the image size
      RCLCPP_INFO(this->get_logger(), "Image dimensions: %d x %d", 
                 image.cols, image.rows);
      
      // Example: Process the image (you can replace this with your own processing)
      cv::imshow("Camera Image", image);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
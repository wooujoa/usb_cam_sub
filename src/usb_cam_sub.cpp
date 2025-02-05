#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    image_transport::ImageTransport it(this->shared_from_this());
    image_sub_ = it.subscribe("image_raw", 10,
      std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try {
      cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
      cv::imshow("USB Camera Image", image);
      cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

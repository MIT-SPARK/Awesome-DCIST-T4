#include <rclcpp/rclcpp.hpp>
#include <ianvs/image_subscription.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace simple_image {
class SimpleImageNode : public rclcpp::Node {
 public:
  explicit SimpleImageNode(const rclcpp::NodeOptions& options)
      : Node("simple_image_node", options), image_sub_(*this) {
    
    // Subscribe to the image topic
    image_sub_.subscribe("image");
    image_sub_.registerCallback(
        std::bind(&SimpleImageNode::imageCallback, this, std::placeholders::_1));
  }

 private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    std::cout << "Received image of size " << image_msg->width << " x " << image_msg->height << "\n";
  }

  ianvs::ImageSubscription image_sub_;
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_image::SimpleImageNode)

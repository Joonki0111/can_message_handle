#include "can_message_handler/can_message_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

CanMessagePublisherNode::CanMessagePublisherNode(const rclcpp::NodeOptions & options)
: Node("can_message_publisher", options)
{
  publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 10,
    std::bind(&CanMessagePublisherNode::canMessageCallback, this, std::placeholders::_1));

  startPublishing();

  RCLCPP_INFO(this->get_logger(), "CanMessagePublisherNode initialized");
}

CanMessagePublisherNode::~CanMessagePublisherNode()
{
  stopPublishing();
  RCLCPP_INFO(this->get_logger(), "CanMessagePublisherNode destroyed");
}

void CanMessagePublisherNode::startPublishing()
{
  is_running_ = true;
  publish_thread_ = std::thread(&CanMessagePublisherNode::publishLoop, this);
}

void CanMessagePublisherNode::stopPublishing()
{
  is_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void CanMessagePublisherNode::publishLoop()
{
  while (is_running_) {
    auto message = state_machine_.generateOutputMessage();
    publisher_->publish(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void CanMessagePublisherNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  state_machine_.updateState(msg);
}

}  // namespace can_message_handler 

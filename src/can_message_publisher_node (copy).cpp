#include "can_message_handler/can_message_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

CanMessagePublisherNode::CanMessagePublisherNode(const rclcpp::NodeOptions & options)
: Node("can_message_publisher", options)
{
  declareParameters();
  can_id_ = this->get_parameter("can_id").as_int();
  is_extended_ = this->get_parameter("is_extended").as_bool();
  dlc_ = this->get_parameter("dlc").as_int();
  period_ms_ = std::chrono::milliseconds(this->get_parameter("period_ms").as_int());

  data_pattern_.resize(dlc_, 0);

  publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  aspc_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    "aspc_status", 10,
    std::bind(&CanMessagePublisherNode::aspcCallback, this, std::placeholders::_1));

  Aspc_ASDrvRdy_ = false;

  startPublishing();

  RCLCPP_INFO(this->get_logger(), "CanMessagePublisherNode initialized");
}

CanMessagePublisherNode::~CanMessagePublisherNode()
{
  stopPublishing();
  RCLCPP_INFO(this->get_logger(), "CanMessagePublisherNode destroyed");
}

void CanMessagePublisherNode::declareParameters()
{
  this->declare_parameter("can_id", 0x001);
  this->declare_parameter("is_extended", false);
  this->declare_parameter("dlc", 8);
  this->declare_parameter("period_ms", 10);
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
    auto message = can_msgs::msg::Frame();
    message.id = can_id_;
    message.is_extended = is_extended_;
    message.dlc = dlc_;

    // Copy data from vector to array
    for (size_t i = 0; i < std::min(dlc_, static_cast<uint8_t>(8)); ++i) {
      message.data[i] = data_pattern_[i];
    }

    if (Aspc_ASDrvRdy_) {
      message.data[0] |= (1 << 2);  // Set bit 2
    } else {
      message.data[0] &= ~(1 << 2);  // Clear bit 2
    }

    publisher_->publish(message);
    std::this_thread::sleep_for(period_ms_);
  }
}

void CanMessagePublisherNode::aspcCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  Aspc_ASDrvRdy_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "ASPC status updated: %s", msg->data ? "true" : "false");
}

}  // namespace can_message_handler 

#include "rclcpp/rclcpp.hpp"
#include "tf_baxter_realsense/srv/calculate_position.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "rclcpp/clock.hpp"
#include <unistd.h>

#include <memory>

class FramePublisher : public rclcpp::Node
{
public:
  rclcpp::Service<tf_baxter_realsense::srv::CalculatePosition>::SharedPtr service;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  FramePublisher()
      : Node("calculate_position")
  {

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

    this->service = this->create_service<tf_baxter_realsense::srv::CalculatePosition>("calculate_position", std::bind(&FramePublisher::calculate, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate.");
  }

private:
  void calculate(const std::shared_ptr<tf_baxter_realsense::srv::CalculatePosition::Request> request,
                 std::shared_ptr<tf_baxter_realsense::srv::CalculatePosition::Response> response)
  {

    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::PoseStamped pose_stamped;

    rclcpp::Rate sleepRate(1.0);

    sleepRate.sleep();
    try
    {

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transformando hacia %s desde %s", request->frame_id.c_str(), request->pose.header.frame_id.c_str());

      transform_stamped = this->tf_buffer->lookupTransform(
          request->frame_id, request->pose.header.frame_id, tf2::TimePointZero);

      tf2::doTransform(request->pose, pose_stamped, transform_stamped);

      response->pose = pose_stamped;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transformada realizada. %f", response->pose.pose.position.x);

      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transformada realizada. %s", response->pose);

    }
    catch (const tf2::TransformException &ex)
    {

      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Problema en la transformada. %s", ex.what());
    }

    //return NULL;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();

  return 0;
}

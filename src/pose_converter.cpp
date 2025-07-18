#include <string>
#include <chrono>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PoseConverter : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    PoseConverter()
    : rclcpp_lifecycle::LifecycleNode(std::string("pose_converter"))
    {

    }
    ~PoseConverter()
    {

    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr posearray_subscriber;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;

    // lifecycle begin
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        // create publisher
        pose_publisher= this->create_publisher<geometry_msgs::msg::Pose2D>(
            std::string("pose"), rclcpp::SystemDefaultsQoS()
        );
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        pose_publisher->on_activate();
        control_timer = this->create_wall_timer(0.01s, std::bind(&PursuitControler::control_callback, this));
        goal_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
            std::string("goal_pose"),
            rclcpp::SystemDefaultsQoS(),
            std::bind(&PursuitControler::goal_callback, this, _1)
        );
        pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
            std::string("pose"),
            rclcpp::SystemDefaultsQoS(),
            std::bind(&PursuitControler::pose_callback, this, _1)
        );
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        pose_publisher->on_deactivate();
        control_timer.reset();
        goal_subscriber.reset();
        pose_subscriber.reset();

        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
        pose_publisher.reset();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_error(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "on error!");
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {
        return CallbackReturn::SUCCESS;
    }
    // lifecycle end
};
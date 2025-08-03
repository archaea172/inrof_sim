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

    /*class func begin*/
    PoseConverter();
    ~PoseConverter();
    /*class func end*/

private:
    /*declare value begin*/
    float x;
    float y;
    float theta;
    /*declare value end*/

    /*node func begin*/
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr posearray_subscriber;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;
    /*node func end*/

    /*lifecycle callback begin*/
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        pose_publisher->on_activate();
        posearray_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
            std::string("/world/inrof_field/pose/info"),
            rclcpp::SystemDefaultsQoS(),
            std::bind(&PoseConverter::posearray_callback, this, _1)
        );
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        pose_publisher->on_deactivate();
        posearray_subscriber.reset();

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

    void posearray_callback(const geometry_msgs::msg::PoseArray::SharedPtr rxdata)
    {
        x       = rxdata->poses[1].position.x;
        y       = rxdata->poses[1].position.y;
        theta   = std::atan2(
            2*(rxdata->poses[1].orientation.w*rxdata->poses[1].orientation.z + rxdata->poses[1].orientation.x*rxdata->poses[1].orientation.y),
            1 - 2*(std::pow(rxdata->poses[1].orientation.y, 2) + std::pow(rxdata->poses[1].orientation.z, 2))
        ) + M_PI/4;

        if (pose_publisher -> is_activated())
        {
            geometry_msgs::msg::Pose2D txdata;
            txdata.x = this->x;
            txdata.y = this->y;
            txdata.theta = this->theta;
            pose_publisher->publish(txdata);
        }
    }
};
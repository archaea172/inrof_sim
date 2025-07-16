#include <string>
#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class TwistConverter : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // constructor
    TwistConverter()
    : rclcpp_lifecycle::LifecycleNode(std::string("twist_converter"))
    {

    }
    // desconstructor
    ~TwistConverter()
    {
        RCLCPP_INFO(this->get_logger(), "finish");
    }
};
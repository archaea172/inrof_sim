#include <string>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class OmniControler : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    OmniControler()
    : rclcpp_lifecycle::LifecycleNode(std::string("omni_controler"))
    {
        
    };
    ~OmniControler()
    {
        RCLCPP_INFO(this->get_logger(), "finish");
    }
};
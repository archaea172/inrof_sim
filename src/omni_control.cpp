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

    // constructor
    OmniControler()
    : rclcpp_lifecycle::LifecycleNode(std::string("omni_controler"))
    {
        
    };

    // desconstructor
    ~OmniControler()
    {
        RCLCPP_INFO(this->get_logger(), "finish");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        joint_pub = this->create_publisher<std_msgs::msg::Float32>(
            std::string("wheel_joint"), rclcpp::SystemDefaultsQoS()
        );
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr joint_pub;

};
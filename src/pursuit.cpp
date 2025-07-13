#include <string>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PursuitControler : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // constructor
    PursuitControler()
    : rclcpp_lifecycle::LifecycleNode(std::string("pursuit_controler"))
    {

    }
    //desconstructor
    ~PursuitControler()
    {

    }
    
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {

    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {

    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {

    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {

    }
    CallbackReturn on_error(const rclcpp_lifecycle::State &state)
    {

    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PursuitControler> node = std::make_shared<PursuitControler>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
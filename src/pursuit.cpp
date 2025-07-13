#include <string>
#include <chrono>
#include <memory>
#include <iostream>

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
        p.resize(3);
        v.resize(3);
        for (int i = 0; i < 3; i++)
        {
            p[i] = 0;
            v[i] = 0;
        }
        T = 30;
    }
    //desconstructor
    ~PursuitControler()
    {

    }

private:
    rclcpp::TimerBase::SharedPtr control_timer;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        control_timer = this->create_wall_timer(1s, std::bind(&PursuitControler::control_callback, this));

        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        control_timer.reset();

        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
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

    void control_callback()
    {
        std::vector<std::vector<float>> p_array(3, std::vector<float>(T, 0));
        std::vector<std::vector<float>> v_array(3, std::vector<float>(T, 0));

        std::tie(p_array, v_array) = predict_position(this->p, this->v);
    }

    std::tuple<std::vector<std::vector<float>>, std::vector<std::vector<float>>>
    predict_position(std::vector<float> p, std::vector<float> v)
    {
        std::vector<std::vector<float>> velocity;
        return std::tie(velocity, velocity);
    }

    float T;// predict horizon

    // current value
    std::vector<float> p;
    std::vector<float> v;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PursuitControler> node = std::make_shared<PursuitControler>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
#include <string>
#include <chrono>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
        dt = 0.001; // ms
    }
    //desconstructor
    ~PursuitControler()
    {

    }

private:

    // node function
    rclcpp::TimerBase::SharedPtr control_timer;

    // lifecycle begin
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
    // lifecycle end

    // control timer callback
    void control_callback()
    {
        std::vector<std::vector<float>> v_array(T, std::vector<float>(3, 0));

        std::vector<std::vector<float>> p_array = predict_position_array(this->p, v_array);
        std::cout << p_array.size() << std::endl;
        std::cout << p_array[0].size() << std::endl;
    }

    //control function
    std::vector<std::vector<float>> predict_position_array(const std::vector<float> &p_value, const std::vector<std::vector<float>> &v_array)
    {
        std::vector<std::vector<float>> p_matrix(T, std::vector<float>(3, 0));
        p_matrix[0] = p_value;
        for (size_t i = 1; i < T; i++) p_matrix[i] = predict_position(p_matrix[i-1], v_array[i]);
        return p_matrix;
    }

    std::vector<float> predict_position(const std::vector<float> &pre_p, const std::vector<float> &current_v)
    {
        std::vector<float> post_p(3);
        for (size_t i = 0; i < 3; i++) post_p[i] = pre_p[i] + dt*current_v[i];
        return post_p;
    }

    // predict horizon
    float T;
    // control cycle
    float dt;
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
#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <random>
#include <Eigen/Dense>

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
        v_ref.resize(2);
        for (int i = 0; i < 3; i++)
        {
            p[i] = 0;
            v[i] = 0;
        }
        for (int i = 0; i < 2; i++)
        {
            v_ref[i] = 0;
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
        std::vector<std::vector<float>> v_array = generate_v_array(3);
        std::vector<std::vector<float>> p_array = predict_position_array(this->p, v_array);


    }

    // estimate function
    float estimate_vel(std::vector<std::vector<float>> V_array)
    {
        std::vector<float> vel_difference;
        vel_difference.resize(T);
        for (int i = 0; i < T; i++)
        {
            vel_difference[i] = std::pow(V_array[i][0], 2) + std::pow(V_array[i][1], 2) - std::pow(v_ref[0], 2);
            vel_difference[i] += std::pow(V_array[i][2] - v_ref[1], 2);
        }
        float difference_sum = 0;
        for (size_t i = 0; i < T; i++) difference_sum += vel_difference[i];
        return difference_sum;
    }
    float estimate_smooth(std::vector<std::vector<float>> V_array)
    {

    }
    float estimate_goal(std::vector<std::vector<float>> X_array)
    {
        
    }

    // probability
    std::vector<std::vector<float>> generate_v_array(const int dim)
    {
        std::vector<std::vector<float>> v_matrix(T, std::vector<float>(3));

        Eigen::VectorXd mu(dim);
        mu << 1.0, 1.0, 0.5;

        Eigen::MatrixXd sigma(dim, dim);
        sigma << 
        1.0, 0.5, 0.2,
        0.5, 1.0, 0.3,
        0.2, 0.3, 1.0;

        std::random_device rd;
        std::mt19937 gen(rd());
        for (size_t i = 0; i < T; i++)
        {
            Eigen::VectorXd v_eigen = sample_multivariate_normal(mu, sigma, gen);
            for (size_t j = 0; j < 3; j++) v_matrix[i][j] = v_eigen[j];
        }
        return v_matrix;
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

    // probability function
    Eigen::VectorXd sample_multivariate_normal(
        const Eigen::VectorXd &mean,
        const Eigen::MatrixXd &cov,
        std::mt19937 &gen
    )
    {
        std::normal_distribution<> dist(0.0, 1.0);
        Eigen::VectorXd z(mean.size());
        for (int i = 0; i < mean.size(); ++i) z(i) = dist(gen);
        Eigen::MatrixXd L = cov.llt().matrixL();
        return mean + L*z;
    }

    // predict horizon
    float T;
    // control cycle
    float dt;
    // goal pose
    std::vector<float> goal_p;
    // current value
    std::vector<float> p;
    std::vector<float> v;
    // vel ref
    std::vector<float> v_ref;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PursuitControler> node = std::make_shared<PursuitControler>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
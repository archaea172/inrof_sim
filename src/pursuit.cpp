#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <math.h>

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
        goal_p.resize(3);
        goal_p[0] = 0;
        goal_p[1] = 0;
        goal_p[2] = 0;
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
        float k_goal_angle;
        float k_goal_linear;
        float k_smooth_angle;
        float k_smooth_wheel;
        float k_smooth_linear;
        float k_vel_angle;
        float k_vel_linear;
        std::vector<std::vector<float>> v_array = generate_v_array(3);
        std::vector<std::vector<float>> p_array = predict_position_array(this->p, v_array);
        float S = k_goal_angle*estimate_goal_angle(p_array)
         + k_goal_linear*estimate_goal_linear(p_array) + k_smooth_angle*estimate_smooth_rotate(v_array) 
         + k_smooth_wheel*estimate_smooth_wheel(p_array, v_array) + k_smooth_linear*estimate_smooth_vel(v_array) 
         + k_vel_linear*estimate_vel(v_array) + k_vel_angle*estimate_vel_rotate(v_array);
        std::cout << S << std::endl;
    }

    // estimate function
    float estimate_vel(std::vector<std::vector<float>> V_array)
    {
        std::vector<float> vel_difference;
        vel_difference.resize(T);
        for (int i = 0; i < T; i++)
        {
            vel_difference[i] = std::pow(V_array[i][0], 2) + std::pow(V_array[i][1], 2) - std::pow(v_ref[0], 2);
        }
        float difference_sum = 0;
        for (size_t i = 0; i < T; i++) difference_sum += vel_difference[i];
        return difference_sum;
    }
    float estimate_vel_rotate(std::vector<std::vector<float>> V_array)
    {
        std::vector<float> vel_difference;
        vel_difference.resize(T);
        for (int i = 0; i < T; i++)
        {
            vel_difference[i] = std::pow(V_array[i][2] - v_ref[1], 2);
        }
        float difference_sum = 0;
        for (size_t i = 0; i < T; i++) difference_sum += vel_difference[i];
        return difference_sum;

    }
    float estimate_smooth_vel(std::vector<std::vector<float>> V_array)
    {
        std::vector<float> smooth_vel_difference(T);
        for (size_t i = 0; i < T - 1; i++)
        {
            float v_ab_sq = std::pow(V_array[i][0], 2) + std::pow(V_array[i][1], 2);
            float v_ab_sq_post = std::pow(V_array[i+1][0], 2) + std::pow(V_array[i+1][1], 2);
            smooth_vel_difference[i] = std::pow(v_ab_sq_post - v_ab_sq, 2);
        }
        float difference_sum_vel = 0;
        for (size_t i = 0; i < T; i++)
        {
            difference_sum_vel += smooth_vel_difference[i];
        }
        return difference_sum_vel;
    }
    float estimate_smooth_rotate(std::vector<std::vector<float>> V_array)
    {
        std::vector<float> smooth_vel_difference(T);
        for (size_t i = 0; i < T - 1; i++)
        {
            smooth_vel_difference[i] += std::pow(V_array[i+1][2] - V_array[i][2], 2);
        }
        float difference_sum_vel = 0;
        for (size_t i = 0; i < T; i++)
        {
            difference_sum_vel += smooth_vel_difference[i];
        }
        return difference_sum_vel;
    }
    float estimate_smooth_wheel(std::vector<std::vector<float>> X_array, std::vector<std::vector<float>> V_array)
    {
        std::vector<float> smooth_wheel_difference(T);
        std::vector<std::vector<float>> wheel_array(T, std::vector<float>(4, 0));
        for (size_t i = 0; i < T; i++)
        {
            float w[4] = {};
            omni_calc(X_array[i][2], V_array[i][0], V_array[i][1], V_array[i][2], &w[0], &w[1], &w[2], &w[3]);
            for (size_t j = 0; j < 4; j++) wheel_array[i][j] = w[j];
        }
        for (size_t i = 0; i < T - 1; i++)
        {
            smooth_wheel_difference[i] = 0;
            for (size_t j = 0; j < 4; j++) smooth_wheel_difference[i] += std::pow(wheel_array[i+1][j] - wheel_array[i][j], 2);
        }
        float difference_sum_wheel = 0;
        for (size_t i = 0; i < T; i++)
        {
            difference_sum_wheel += smooth_wheel_difference[i];
        }
        return difference_sum_wheel;
    }
    float estimate_goal_linear(std::vector<std::vector<float>> X_array)
    {
        std::vector<float> differential_goal(T);
        for (size_t i = 0; i < T; i++)
        {
            differential_goal[i] = std::pow(goal_p[0] - X_array[i][0], 2) + std::pow(goal_p[1] - X_array[i][1], 2) + std::pow(goal_p[2] - X_array[i][2], 2);
        }
        float difference_sum = 0;
        for (size_t i = 0; i < T; i++) difference_sum += differential_goal[i];
        return difference_sum;
    }
    float estimate_goal_angle(std::vector<std::vector<float>> X_array)
    {
        std::vector<float> differential_goal(T);
        for (size_t i = 0; i < T; i++)
        {
            differential_goal[i] = std::pow(goal_p[2] - X_array[i][2], 2);
        }
        float difference_sum = 0;
        for (size_t i = 0; i < T; i++) difference_sum += differential_goal[i];
        return difference_sum;
    }

    // omni_simulate
    void omni_calc(float theta,float vx,float vy,float omega,float *w0,float *w1,float *w2,float *w3){
        const float a0 = M_PI/180*45;
        const float a1 = M_PI/180*135;
        const float a2 = M_PI/180*225;
        const float a3 = M_PI/180*315;
        const float r = 0.03;//m
        const float R = 0.144;//m
        float v[3] = {vx, vy, omega};
        float sint = sin(theta);
        float cost = cos(theta);

        float arr[4][3] =
        {{-cos(a0)*sint-sin(a0)*cost, cos(a0)*cost-sin(a0)*sint, R},
        {-cos(a1)*sint-sin(a1)*cost, cos(a1)*cost-sin(a1)*sint, R},
        {-cos(a2)*sint-sin(a2)*cost, cos(a2)*cost-sin(a2)*sint, R},
        {-cos(a3)*sint-sin(a3)*cost, cos(a3)*cost-sin(a3)*sint, R}};

        *w0 = (arr[0][0] * v[0] + arr[0][1] * v[1] + arr[0][2] * v[2]) / r;
        *w1 = (arr[1][0] * v[0] + arr[1][1] * v[1] + arr[1][2] * v[2]) / r;
        *w2 = (arr[2][0] * v[0] + arr[2][1] * v[1] + arr[2][2] * v[2]) / r;
        *w3 = (arr[3][0] * v[0] + arr[3][1] * v[1] + arr[3][2] * v[2]) / r;
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
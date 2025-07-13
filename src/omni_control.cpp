#include <string>
#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

const float a0 = M_PI/180*45;
const float a1 = M_PI/180*135;
const float a2 = M_PI/180*225;
const float a3 = M_PI/180*315;
const float r = 0.03;//m
const float R = 0.144;//m

void omni_calc(float theta,float vx,float vy,float omega,float *w0,float *w1,float *w2,float *w3){

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
        joint_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            std::string("wheel_joint"), rclcpp::SystemDefaultsQoS()
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        // activate publisher
        joint_pub->on_activate();

        // create subscriber
        vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            std::string("cmd_vel"),
            rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniControler::vel_callback, this, _1)
        );
        pub_timer = this->create_wall_timer(0.001s, std::bind(&OmniControler::timer_callback, this));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        // deactivate publisher
        joint_pub->on_deactivate();
        // reset subscriber
        vel_subscriber.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
        // reset publisher
        joint_pub.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "on error!", state.label().c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {
        return CallbackReturn::SUCCESS;
    }

    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr rxdata) const
    {
        if (joint_pub->is_activated())
        {
            float theta = 0;
            float vx = rxdata->linear.x;
            float vy = rxdata->linear.y;
            float omega = rxdata->angular.z;

            // publish
            float w[4] = {0, 0, 0, 0};
            omni_calc(theta, vx, vy, omega, &w[0], &w[1], &w[2], &w[3]);
            std_msgs::msg::Float32MultiArray txdata;
            std::vector<float> w_vector(4);
            txdata.data.resize(4);
            for (int i = 0; i < 4; i++) w_vector[i] = w[i];
            txdata.data = w_vector;
            joint_pub->publish(txdata);
        }
    }

    void timer_callback()
    {

    }

private:
    // define publisher
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub;
    // define subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;
    // define timer
    rclcpp::TimerBase::SharedPtr pub_timer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<OmniControler> node = std::make_shared<OmniControler>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
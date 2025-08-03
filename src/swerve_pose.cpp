#include "rclcpp/rclcpp.hpp"
#include "swerve_pose.hpp"

/*class func begin*/
PoseConverter::PoseConverter()
: rclcpp_lifecycle::LifecycleNode(std::string("pose_converter"))
{
    pose_publisher= this->create_publisher<geometry_msgs::msg::Pose2D>(
        std::string("pose"), rclcpp::SystemDefaultsQoS()
    );
}

PoseConverter::~PoseConverter()
{

}
/*class func end*/

/*lifecycle callback begin*/    
PoseConverter::CallbackReturn PoseConverter::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

PoseConverter::CallbackReturn PoseConverter::on_activate(const rclcpp_lifecycle::State &state)
{
    pose_publisher->on_activate();
    posearray_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        std::string("/world/swerve_sim/pose/info"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&PoseConverter::posearray_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

PoseConverter::CallbackReturn PoseConverter::on_deactivate(const rclcpp_lifecycle::State &state)
{
    pose_publisher->on_deactivate();
    posearray_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

PoseConverter::CallbackReturn PoseConverter::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

PoseConverter::CallbackReturn PoseConverter::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "on error!");
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}
/*lifecycle callback end*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PoseConverter> node = std::make_shared<PoseConverter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
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

PoseConverter::CallbackReturn PoseConverter::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}
/*lifecycle callback end*/

/*callback begin*/
void PoseConverter::posearray_callback(const geometry_msgs::msg::PoseArray::SharedPtr rxdata)
{
    x       = rxdata->poses[3].position.x;
    y       = rxdata->poses[3].position.y;
    theta   = std::atan2(
        2*(rxdata->poses[3].orientation.w*rxdata->poses[3].orientation.z + rxdata->poses[3].orientation.x*rxdata->poses[3].orientation.y),
        1 - 2*(std::pow(rxdata->poses[3].orientation.y, 2) + std::pow(rxdata->poses[3].orientation.z, 2))
    ) + M_PI/4;

    if (pose_publisher -> is_activated())
    {
        geometry_msgs::msg::Pose2D txdata;
        txdata.x = this->x;
        txdata.y = this->y;
        txdata.theta = this->theta;
        pose_publisher->publish(txdata);
    }
}
/*callback end*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PoseConverter> node = std::make_shared<PoseConverter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "swerve_pose.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PoseConverter> node = std::make_shared<PoseConverter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
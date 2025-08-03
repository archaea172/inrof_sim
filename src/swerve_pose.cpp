#include "rclcpp/rclcpp.hpp"
#include "swerve_pose.hpp"

/*class func begin*/
PoseConverter::PoseConverter()
: rclcpp_lifecycle::LifecycleNode(std::string("pose_converter"))
{

}

PoseConverter::~PoseConverter()
{

}
/*class func end*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PoseConverter> node = std::make_shared<PoseConverter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
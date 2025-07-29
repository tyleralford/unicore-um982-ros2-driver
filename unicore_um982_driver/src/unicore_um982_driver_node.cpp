#include <rclcpp/rclcpp.hpp>

class UnicoreDriverNode : public rclcpp::Node
{
public:
    UnicoreDriverNode() : Node("unicore_um982_driver")
    {
        RCLCPP_INFO(this->get_logger(), "Unicore UM982 Driver Node started");
    }

private:
    // Node implementation will be expanded in subsequent tasks
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnicoreDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

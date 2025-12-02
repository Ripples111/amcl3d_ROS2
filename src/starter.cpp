#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <iostream>
#include <vector>

// class KeyboardPublisher : public rclcpp::Node
// {
// public:
//     KeyboardPublisher() : Node("starter_node")
//     {
//         RCLCPP_INFO(this->get_logger(), "节点已启动");
//         // 创建一个发布者，发布到 "keyboard_input" 话题
//         publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/initial_pose", 10);
//     }

//     void publish_numbers()
//     {
//         std_msgs::msg::Float64MultiArray msg;

//         // 提示用户输入四个数字
//         std::cout << "请输入初始位姿(X,Y,Z,A): "<< std::flush;
//         std::vector<double> numbers(4);
//         for (size_t i = 0; i < 4; ++i)
//         {
//             std::cin >> numbers[i];
//         }

//         // 将输入的数字添加到消息中
//         msg.data = numbers;

//         // 发布消息
//         publisher_->publish(msg);
//         //RCLCPP_INFO(this->get_logger(), "已发布数字: [%.2f, %.2f, %.2f, %.2f]", numbers[0], numbers[1], numbers[2], numbers[3]);
//     }

// private:
//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
// };

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::NodeOptions options;
    // options.use_intra_process_comms(true);
    // rclcpp::executors::SingleThreadedExecutor exec;
    //auto node = std::make_shared<KeyboardPublisher>();
    //exec.add_node(node1);  
    //exec.spin();

    //node->publish_numbers();

    rclcpp::shutdown();
    return 0;
}
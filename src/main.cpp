/*!
 * @file main.cpp
 * @copyright Copyright (c) 2019, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::executors::SingleThreadedExecutor exec;
  //auto node = std::make_shared<rclcpp::Node>("amcl3d_node");
  auto node2 = std::make_shared<amcl3d::Amcl>("amcl3d_node");
  exec.add_node(node2);

  RCLCPP_INFO(node2->get_logger(), "[%s] Node initialization.", node2->get_name());
  exec.spin();
  
  rclcpp::shutdown();
  RCLCPP_INFO(node2->get_logger(), "[%s] Node finished.", node2->get_name());

  return EXIT_SUCCESS;
}

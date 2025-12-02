/*!
 * @file Parameters.cpp
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

#include "Parameters.h"

#include <rclcpp/rclcpp.hpp>

namespace amcl3d
{
Parameters::Parameters() : rclcpp::Node("amcl3d_node_parameters")
{
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "default_value");
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  global_frame_id_ = this->declare_parameter<std::string>("global_frame_id", "world");
  map_path_ = this->declare_parameter<std::string>("map_path", "default_value");
  set_initial_pose_ = this->declare_parameter<bool>("set_initial_pose", true);
  init_x_ = this->declare_parameter<double>("init_x", 0.0);
  init_y_ = this->declare_parameter<double>("init_y", 0.0);
  init_z_ = this->declare_parameter<double>("init_z", 0.0);
  init_a_ = this->declare_parameter<double>("init_a", 0.0);
  init_x_dev_ = this->declare_parameter<double>("init_x_dev", 0.05);
  init_y_dev_ = this->declare_parameter<double>("init_y_dev", 0.05);
  init_z_dev_ = this->declare_parameter<double>("init_z_dev", 0.05);
  init_a_dev_ = this->declare_parameter<double>("init_a_dev", 0.1);
  publish_point_cloud_rate_ = this->declare_parameter<double>("publish_point_cloud_rate", 10);
  grid_slice_z_ = this->declare_parameter<double>("grid_slice_z", -1.0);
  publish_grid_slice_rate_ = this->declare_parameter<double>("publish_grid_slice_rate", 10);
  sensor_dev_ = this->declare_parameter<double>("sensor_dev", 0.05);
  sensor_range_ = this->declare_parameter<double>("sensor_range", 0.0);
  voxel_size_ = this->declare_parameter<double>("voxel_size", 0.1);
  num_particles_ = this->declare_parameter<int>("num_particles", 600);
  odom_x_mod_ = this->declare_parameter<double>("odom_x_mod", 0.1);
  odom_y_mod_ = this->declare_parameter<double>("odom_y_mod", 0.1);
  odom_z_mod_ = this->declare_parameter<double>("odom_z_mod", 0.01);
  odom_a_mod_ = this->declare_parameter<double>("odom_a_mod", 0.3);
  resample_interval_ = this->declare_parameter<int>("resample_interval", 0);
  update_rate_ =this->declare_parameter<int>("update_rate", 100);
  d_th_ =this->declare_parameter<double>("d_th", 0.05);
  a_th_ =this->declare_parameter<double>("a_th", 0.03);
  take_off_height_ =this->declare_parameter<double>("take_off_height", 0.0);
  alpha_ =this->declare_parameter<double>("alpha", 1.0);

  if (base_frame_id_ == "default_value")
  {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'base_frame_id' not set");
      exitWithParameterError("base_frame_id");
  }

  // if (!this->get_parameter("~odom_frame_id", odom_frame_id_))
  // {
  //   exitWithParameterError("odom_frame_id");
  // }

  // if (!this->get_parameter("~global_frame_id", global_frame_id_))
  // {
  //   exitWithParameterError("global_frame_id");
  // }

  // if (!this->get_parameter("~map_path", map_path_))
  // {
  //   exitWithParameterError("map_path");
  // }

  // if (!this->get_parameter("~set_initial_pose", set_initial_pose_))
  // {
  //   exitWithParameterError("set_initial_pose");
  // }

  // if (!this->get_parameter("~init_x", init_x_))
  // {
  //   exitWithParameterError("init_x");
  // }

  // if (!this->get_parameter("~init_y", init_y_))
  // {
  //   exitWithParameterError("init_y");
  // }

  // if (!this->get_parameter("~init_z", init_z_))
  // {
  //   exitWithParameterError("init_z");
  // }

  // if (!this->get_parameter("~init_a", init_a_))
  // {
  //   exitWithParameterError("init_a");
  // }

  // if (!this->get_parameter("~init_x_dev", init_x_dev_))
  // {
  //   exitWithParameterError("init_x_dev");
  // }

  // if (!this->get_parameter("~init_y_dev", init_y_dev_))
  // {
  //   exitWithParameterError("init_y_dev");
  // }

  // if (!this->get_parameter("~init_z_dev", init_z_dev_))
  // {
  //   exitWithParameterError("init_z_dev");
  // }

  // if (!this->get_parameter("~init_a_dev", init_a_dev_))
  // {
  //   exitWithParameterError("init_a_dev");
  // }

  // if (!this->get_parameter("~publish_point_cloud_rate", publish_point_cloud_rate_))
  // {
  //   exitWithParameterError("publish_point_cloud_rate");
  // }

  // if (!this->get_parameter("~grid_slice_z", grid_slice_z_))
  // {
  //   exitWithParameterError("grid_slice_z");
  // }

  // if (!this->get_parameter("~publish_grid_slice_rate", publish_grid_slice_rate_))
  // {
  //   exitWithParameterError("publish_grid_slice_rate");
  // }

  // if (!this->get_parameter("~sensor_dev", sensor_dev_))
  // {
  //   exitWithParameterError("sensor_dev");
  // }

  // if (!this->get_parameter("~sensor_range", sensor_range_))
  // {
  //   exitWithParameterError("sensor_range");
  // }

  // if (!this->get_parameter("~voxel_size", voxel_size_))
  // {
  //   exitWithParameterError("voxel_size");
  // }

  // if (!this->get_parameter("~num_particles", num_particles_))
  // {
  //   exitWithParameterError("num_particles");
  // }

  // if (!this->get_parameter("~odom_x_mod", odom_x_mod_))
  // {
  //   exitWithParameterError("odom_x_mod");
  // }

  // if (!this->get_parameter("~odom_y_mod", odom_y_mod_))
  // {
  //   exitWithParameterError("odom_y_mod");
  // }

  // if (!this->get_parameter("~odom_z_mod", odom_z_mod_))
  // {
  //   exitWithParameterError("odom_z_mod");
  // }

  // if (!this->get_parameter("~odom_a_mod", odom_a_mod_))
  // {
  //   exitWithParameterError("odom_a_mod");
  // }

  // if (!this->get_parameter("~resample_interval", resample_interval_))
  // {
  //   exitWithParameterError("resample_interval");
  // }

  // if (!this->get_parameter("~update_rate", update_rate_))
  // {
  //   exitWithParameterError("update_rate");
  // }

  // if (!this->get_parameter("~d_th", d_th_))
  // {
  //   exitWithParameterError("d_th");
  // }

  // if (!this->get_parameter("~a_th", a_th_))
  // {
  //   exitWithParameterError("a_th");
  // }

  // if (!this->get_parameter("~take_off_height", take_off_height_))
  // {
  //   exitWithParameterError("take_off_height");
  // }

  // if (!this->get_parameter("~alpha", alpha_))
  // {
  //   exitWithParameterError("alpha");
  // }

  RCLCPP_INFO(this->get_logger(), 
          "[%s]"
           "\n      Parameters:"
           "\n      base_frame_id=%s"
           "\n      odom_frame_id=%s"
           "\n      global_frame_id=%s"
           "\n      map_path=%s"
           "\n      set_initial_pose=%d"
           "\n      init_x=%lf"
           "\n      init_y=%lf"
           "\n      init_z=%lf"
           "\n      init_a=%lf"
           "\n      init_x_dev=%lf"
           "\n      init_y_dev=%lf"
           "\n      init_z_dev=%lf"
           "\n      init_a_dev=%lf"
           "\n      publish_point_cloud_rate=%lf"
           "\n      grid_slice_z=%f"
           "\n      publish_grid_slice_rate=%lf"
           "\n      sensor_dev=%lf"
           "\n      sensor_range=%lf"
           "\n      voxel_size=%lf"
           "\n      num_particles=%d"
           "\n      odom_x_mod=%lf"
           "\n      odom_y_mod=%lf"
           "\n      odom_z_mod=%lf"
           "\n      odom_a_mod=%lf"
           "\n      resample_interval=%d"
           "\n      update_rate=%lf"
           "\n      d_th=%lf"
           "\n      a_th=%lf"
           "\n      take_off_height=%lf"
           "\n      alpha=%lf",
           this->get_name(), base_frame_id_.c_str(), odom_frame_id_.c_str(), global_frame_id_.c_str(),
           map_path_.c_str(), (int)set_initial_pose_, init_x_, init_y_, init_z_, init_a_, init_x_dev_, init_y_dev_,
           init_z_dev_, init_a_dev_, publish_point_cloud_rate_, grid_slice_z_, publish_grid_slice_rate_, sensor_dev_,
           sensor_range_, voxel_size_, num_particles_, odom_x_mod_, odom_y_mod_, odom_z_mod_, odom_a_mod_,
           resample_interval_, update_rate_, d_th_, a_th_, take_off_height_, alpha_);
}

void Parameters::exitWithParameterError(const char* parameter_str)
{
  RCLCPP_INFO(this->get_logger(), "[%s] `%s` parameter not set!", this->get_name(), parameter_str);
  exit(EXIT_FAILURE);
}

}  // namespace amcl3d

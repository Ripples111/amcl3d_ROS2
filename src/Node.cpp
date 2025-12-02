/*!
 * @file Node.cpp
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
#include "Grid3d.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace amcl3d
{
Amcl::Amcl(const std::string& node_name) : rclcpp::Node(node_name)
{

  //RCLCPP_DEBUG(this->get_logger(), "Node::Node()");
  // 初始化其他成员变量
  std::shared_ptr<amcl3d::Grid3d> grid3d_ = std::make_shared<Grid3d>();
  std::shared_ptr<amcl3d::ParticleFilter> pf_ = std::make_shared<ParticleFilter>();

  //pointcloudCallback2();

  if (!grid3d_->open(parameters_.map_path_, parameters_.sensor_dev_))
  return;

  if (parameters_.publish_grid_slice_rate_ != 0 &&
      grid3d_->buildGridSliceMsg(parameters_.grid_slice_z_, grid_slice_msg_))
  {
    grid_slice_msg_.header.frame_id = parameters_.global_frame_id_;
    grid_slice_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_slice",rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    grid_slice_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / parameters_.publish_grid_slice_rate_)),
            std::bind(&Amcl::publishGridSlice, this));
  }

  if (parameters_.publish_point_cloud_rate_ != 0 && grid3d_->buildMapPointCloudMsg(map_point_cloud_msg_))
  {
    map_point_cloud_msg_.header.frame_id = parameters_.global_frame_id_;
    map_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_point_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    map_point_cloud_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / parameters_.publish_point_cloud_rate_)),
            std::bind(&Amcl::publishGridSlice, this));                                             
  }

  point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&Amcl::pointcloudCallback2, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/odom", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&Amcl::odomCallback, this, std::placeholders::_1));
  // range_sub_ = this->create_subscription<rosinrange_msg::msg::RangePose>("/radiorange_sensor", rclcpp::QoS(rclcpp::KeepLast(1)),
  //           std::bind(&Amcl::rangeCallback, this, std::placeholders::_1));
  initial_pose_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/initial_pose",rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&Amcl::get_initial_pose, this, std::placeholders::_1));

  particles_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  range_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("range", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  odom_base_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("base_transform", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  cloud_filter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_filtered", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

}

Amcl::~Amcl()
{
    //RCLCPP_DEBUG(this->get_logger(), "Node::~Node()");
    // 清理资源
}

void Amcl::spin()
{
  //RCLCPP_INFO(this->get_logger(),"[%s] Node::spin()",this->get_name());

  // if (!grid3d_.open(parameters_.map_path_, parameters_.sensor_dev_))
  //   return;

  // if (parameters_.publish_grid_slice_rate_ != 0 &&
  //     grid3d_.buildGridSliceMsg(parameters_.grid_slice_z_, grid_slice_msg_))
  // {
  //   grid_slice_msg_.header.frame_id = parameters_.global_frame_id_;
  //   grid_slice_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_slice",rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  //   grid_slice_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / parameters_.publish_grid_slice_rate_)),
  //           std::bind(&Amcl::publishGridSlice, this));
  // }

  // if (parameters_.publish_point_cloud_rate_ != 0 && grid3d_.buildMapPointCloudMsg(map_point_cloud_msg_))
  // {
  //   map_point_cloud_msg_.header.frame_id = parameters_.global_frame_id_;
  //   map_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_point_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  //   map_point_cloud_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / parameters_.publish_point_cloud_rate_)),
  //           std::bind(&Amcl::publishGridSlice, this));                                             
  // }

  // point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)),
  //           std::bind(&Amcl::pointcloudCallback2, this, std::placeholders::_1));
  // odom_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/odom", rclcpp::QoS(rclcpp::KeepLast(1)),
  //           std::bind(&Amcl::odomCallback, this, std::placeholders::_1));
  // // range_sub_ = this->create_subscription<rosinrange_msg::msg::RangePose>("/radiorange_sensor", rclcpp::QoS(rclcpp::KeepLast(1)),
  // //           std::bind(&Amcl::rangeCallback, this, std::placeholders::_1));
  // initial_pose_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/initial_pose",rclcpp::QoS(rclcpp::KeepLast(1)),
  //           std::bind(&Amcl::get_initial_pose, this, std::placeholders::_1));

  // particles_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  // range_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("range", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  // odom_base_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("base_transform", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // cloud_filter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_filtered", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   usleep(100);
  // }

  //this->shutdown();
  
}

void Amcl::publishMapPointCloud()
{
  RCLCPP_DEBUG(this->get_logger(),"[%s] Node::publishMapPointCloud()", this->get_name());

  map_point_cloud_msg_.header.stamp = this->now();
  map_point_cloud_pub_->publish(map_point_cloud_msg_);
}

void Amcl::publishGridSlice()
{
  RCLCPP_DEBUG(this->get_logger(),"[%s] Node::publishGridSlice()", this->get_name());

  grid_slice_msg_.header.stamp = this->now();
  grid_slice_pub_->publish(grid_slice_msg_);
}

void Amcl::publishParticles()
{
  /* If the filter is not initialized then exit */
  if (!pf_.isInitialized())
    return;

  /* Build the msg based on the particles position and orientation */
  geometry_msgs::msg::PoseArray msg;
  pf_.buildParticlesPoseMsg(msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = parameters_.global_frame_id_;

  /* Publish particle cloud */
  particles_pose_pub_->publish(msg);
}

void Amcl::get_initial_pose(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    parameters_.init_x_ = msg->data[0];
    parameters_.init_y_ = msg->data[1];
    parameters_.init_z_ = msg->data[2];
    parameters_.init_a_ = msg->data[3];
    std::cout<<"pose recived"<<std::endl;
    get_initial_pose_ = true;

}

void Amcl::pointcloudCallback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::cout<<"Point processsing"<<std::endl;
  RCLCPP_DEBUG(this->get_logger(),"pointcloudCallback open");

  if (!is_odom_)
  {
    RCLCPP_WARN(this->get_logger(),"Odometry transform not received");
    return;
  }

  /* Check if an update must be performed or not */
  if (!checkUpdateThresholds())
    return;

  #include <chrono>

  static const auto update_interval = std::chrono::milliseconds(static_cast<int>(1000.0 / parameters_.update_rate_));
  nextupdate_time_ = this->now() + update_interval;

  /* Apply voxel grid */
  clock_t begin_filter = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud_src);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_src);
  sor.setLeafSize(parameters_.voxel_size_, parameters_.voxel_size_, parameters_.voxel_size_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*cloud_down);
  cloud_down->header = cloud_src->header;
  sensor_msgs::msg::PointCloud2 cloud_down_msg;
  pcl::toROSMsg(*cloud_down, cloud_down_msg);
  cloud_filter_pub_->publish(cloud_down_msg);
  clock_t end_filter = clock();
  double elapsed_secs = double(end_filter - begin_filter) / CLOCKS_PER_SEC;
  RCLCPP_DEBUG(this->get_logger(),"Filter time: [%lf] sec", elapsed_secs);

  /* Perform particle prediction based on odometry */
  odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;
  const double delta_x = odom_increment_tf_.getOrigin().getX();
  const double delta_y = odom_increment_tf_.getOrigin().getY();
  const double delta_z = odom_increment_tf_.getOrigin().getZ();
  const double delta_a = getYawFromTf(odom_increment_tf_);

  clock_t begin_predict = clock();
  pf_.predict(parameters_.odom_x_mod_, parameters_.odom_y_mod_, parameters_.odom_z_mod_, parameters_.odom_a_mod_,
              delta_x, delta_y, delta_z, delta_a);
  clock_t end_predict = clock();
  elapsed_secs = double(end_predict - begin_predict) / CLOCKS_PER_SEC;
  RCLCPP_DEBUG(this->get_logger(),"Predict time: [%lf] sec", elapsed_secs);

  // /* Perform particle update based on current point-cloud */
  clock_t begin_update = clock();
  //pf_.update(grid3d_, cloud_down, range_data, parameters_.alpha_, parameters_.sensor_range_, roll_, pitch_);
  pf_.update(grid3d_, cloud_down, parameters_.alpha_, parameters_.sensor_range_, roll_, pitch_);
  clock_t end_update = clock();
  elapsed_secs = double(end_update - begin_update) / CLOCKS_PER_SEC;
  RCLCPP_DEBUG(this->get_logger(),"Update time: [%lf] sec", elapsed_secs);

  mean_p_ = pf_.getMean();
  std::ofstream file;
  file.open("/home/wan/ROS2_code/mcl/src/amcl3d/204_odom_macl.csv", std::ios::app);  // 以追加模式打开文件
  if (file.is_open()) {
      std::cout<<"file opened"<<std::endl;
      file << mean_p_.x << "," << mean_p_.y << "," << mean_p_.z << ","<<mean_p_.a<<std::endl;  // 位置
            //<< qx << "," << qy << "," << qz << "," << qw << std::endl;  // 姿态
      file.close();
  } else {
      
  }
  std::cout<<"x is "<<mean_p_.x<<std::endl;
  std::cout<<"y is "<<mean_p_.y<<std::endl;
  std::cout<<"z is "<<mean_p_.z<<std::endl;
  std::cout<<"w is "<<mean_p_.w<<std::endl;

  /* Clean the range buffer */
  range_data.clear();

  /* Update time and transform information */
  lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

  /* Do the resampling if needed */
  clock_t begin_resample = clock();
  static int n_updates = 0;
  if (++n_updates > parameters_.resample_interval_)
  {
    n_updates = 0;
    pf_.resample();
  }
  clock_t end_resample = clock();
  elapsed_secs = double(end_resample - begin_resample) / CLOCKS_PER_SEC;
  RCLCPP_DEBUG(this->get_logger(),"Resample time: [%lf] sec", elapsed_secs);

  /* Publish particles */
  publishParticles();

  RCLCPP_DEBUG(this->get_logger(),"pointcloudCallback close");
}

void Amcl::odomCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(),"odomCallback open");

  base_2_odom_tf_.setOrigin(tf2::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  base_2_odom_tf_.setRotation(tf2::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                                             msg->transform.rotation.z, msg->transform.rotation.w));

  if(!get_initial_pose_)
  {
    RCLCPP_WARN(this->get_logger(),"Initial pose not entered.");
    return ;
  }        
  
  /* If the filter is not initialized then exit */
  if (!pf_.isInitialized())
  {
    RCLCPP_WARN(this->get_logger(),"Filter not initialized yet, waiting for initial pose.");
    if (parameters_.set_initial_pose_)
    {
      tf2::Transform init_pose;
      init_pose.setOrigin(tf2::Vector3(parameters_.init_x_, parameters_.init_y_, parameters_.init_z_));
      init_pose.setRotation(tf2::Quaternion(0.0, 0.0, sin(parameters_.init_a_ * 0.5), cos(parameters_.init_a_ * 0.5)));
      setInitialPose(init_pose, parameters_.init_x_dev_, parameters_.init_y_dev_, parameters_.init_z_dev_,
                     parameters_.init_a_dev_);
    }
    return;
  }

  /* Update roll and pitch from odometry */
  double yaw;
  base_2_odom_tf_.getBasis().getRPY(roll_, pitch_, yaw);

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br;
  // tf_br->sendTransform(
  //     tf2::StampedTransform(base_2_odom_tf_, this->now(), parameters_.odom_frame_id_, parameters_.base_frame_id_));
  geometry_msgs::msg::TransformStamped transform_stamped;
  // 设置消息头
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = parameters_.odom_frame_id_;
  transform_stamped.child_frame_id = parameters_.base_frame_id_;

  // 设置变换信息
  transform_stamped.transform.translation.x = base_2_odom_tf_.getOrigin().x();
  transform_stamped.transform.translation.y = base_2_odom_tf_.getOrigin().y();
  transform_stamped.transform.translation.z = base_2_odom_tf_.getOrigin().z();

  // 设置旋转（四元数）
  transform_stamped.transform.rotation.x = base_2_odom_tf_.getRotation().x();
  transform_stamped.transform.rotation.y = base_2_odom_tf_.getRotation().y();
  transform_stamped.transform.rotation.z = base_2_odom_tf_.getRotation().z();
  transform_stamped.transform.rotation.w = base_2_odom_tf_.getRotation().w();

  // 发布变换信息
  tf_br->sendTransform(transform_stamped);

  if (!is_odom_)
  {
    is_odom_ = true;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;
  }

  static bool has_takenoff = false;
  if (!has_takenoff)
  {
    RCLCPP_WARN(this->get_logger(),"Not <<taken off>> yet");

    /* Check takeoff height */
    has_takenoff = base_2_odom_tf_.getOrigin().getZ() > parameters_.take_off_height_;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;

    lastmean_p_ = mean_p_;  // for not 'jumping' whenever has_takenoff is true */
  }
  else
  {
    /* Check if AMCL went wrong (nan, inf) */
    if (std::isnan(mean_p_.x) || std::isnan(mean_p_.y) || std::isnan(mean_p_.z) || std::isnan(mean_p_.a))
    {
      RCLCPP_WARN(this->get_logger(),"AMCL NaN detected");
      amcl_out_ = true;
    }
    if (std::isinf(mean_p_.x) || std::isinf(mean_p_.y) || std::isinf(mean_p_.z) || std::isinf(mean_p_.a))
    {
      RCLCPP_WARN(this->get_logger(),"AMCL Inf detected");
      amcl_out_ = true;
    }

    /* Check jumps */
    if (fabs(mean_p_.x - lastmean_p_.x) > 1.)
    {
      RCLCPP_WARN(this->get_logger(),"AMCL Jump detected in X");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.y - lastmean_p_.y) > 1.)
    {
      RCLCPP_WARN(this->get_logger(),"AMCL Jump detected in Y");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.z - lastmean_p_.z) > 1.)
    {
      RCLCPP_WARN(this->get_logger(),"AMCL Jump detected in Z");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.a - lastmean_p_.a) > 1.)
    {
      RCLCPP_WARN(this->get_logger(),"AMCL Jump detected in Yaw");
      amcl_out_ = true;
    }

    if (!amcl_out_)
    {
      tf2::Transform base_2_world_tf;
      base_2_world_tf.setOrigin(tf2::Vector3(mean_p_.x, mean_p_.y, mean_p_.z));
      tf2::Quaternion q;
      q.setRPY(roll_, pitch_, mean_p_.a);
      base_2_world_tf.setRotation(q);

      base_2_world_tf = base_2_world_tf * lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

      lastmean_p_ = mean_p_;

      lastbase_2_world_tf_ = base_2_world_tf;
      lastodom_2_world_tf_ = base_2_world_tf * base_2_odom_tf_.inverse();

      amcl_out_lastbase_2_odom_tf_ = lastupdatebase_2_odom_tf_;
    }
    else
    {
      lastbase_2_world_tf_ = lastbase_2_world_tf_ * amcl_out_lastbase_2_odom_tf_.inverse() * base_2_odom_tf_;
      amcl_out_lastbase_2_odom_tf_ = base_2_odom_tf_;
    }
  }

  /* Publish transform */
  geometry_msgs::msg::TransformStamped odom_2_base_tf;
  odom_2_base_tf.header.stamp = msg->header.stamp;
  odom_2_base_tf.header.frame_id = parameters_.global_frame_id_;
  odom_2_base_tf.child_frame_id = parameters_.base_frame_id_;
  odom_2_base_tf.transform.translation.x = lastbase_2_world_tf_.getOrigin().getX();
  odom_2_base_tf.transform.translation.y = lastbase_2_world_tf_.getOrigin().getY();
  odom_2_base_tf.transform.translation.z = lastbase_2_world_tf_.getOrigin().getZ();
  odom_2_base_tf.transform.rotation.x = lastbase_2_world_tf_.getRotation().getX();
  odom_2_base_tf.transform.rotation.y = lastbase_2_world_tf_.getRotation().getY();
  odom_2_base_tf.transform.rotation.z = lastbase_2_world_tf_.getRotation().getZ();
  odom_2_base_tf.transform.rotation.w = lastbase_2_world_tf_.getRotation().getW();
  odom_base_pub_->publish(odom_2_base_tf);

  // tf_br.sendTransform(tf2::StampedTransform(lastodom_2_world_tf_, this->now(), parameters_.global_frame_id_,
  //                                          parameters_.odom_frame_id_));
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = parameters_.global_frame_id_;
  transform_stamped.child_frame_id = parameters_.odom_frame_id_;

  // 设置变换信息
  transform_stamped.transform.translation.x = lastodom_2_world_tf_.getOrigin().x();
  transform_stamped.transform.translation.y = lastodom_2_world_tf_.getOrigin().y();
  transform_stamped.transform.translation.z = lastodom_2_world_tf_.getOrigin().z();

  // 设置旋转（四元数）
  transform_stamped.transform.rotation.x = lastodom_2_world_tf_.getRotation().x();
  transform_stamped.transform.rotation.y = lastodom_2_world_tf_.getRotation().y();
  transform_stamped.transform.rotation.z = lastodom_2_world_tf_.getRotation().z();
  transform_stamped.transform.rotation.w = lastodom_2_world_tf_.getRotation().w();

  // 发布变换信息
  tf_br->sendTransform(transform_stamped);

  RCLCPP_DEBUG(this->get_logger(),"odomCallback close");
}

// void Amcl::rangeCallback(const rosinrange_msg::msg::RangePose::SharedPtr msg)
// {
//   RCLCPP_DEBUG(this->get_logger(),"rangeCallback open");

//   geometry_msgs::msg::Point anchor;
//   anchor.x = msg->position.x;
//   anchor.y = msg->position.y;
//   anchor.z = msg->position.z;

//   range_data.push_back(Range(static_cast<float>(msg->range), msg->position.x, msg->position.y, msg->position.z));

//   geometry_msgs::msg::Point uav;
//   uav.x = mean_p_.x;
//   uav.y = mean_p_.y;
//   uav.z = mean_p_.z;

//   rvizMarkerPublish(msg->source_id, static_cast<float>(msg->range), uav, anchor);

//   RCLCPP_DEBUG(this->get_logger(),"rangeCallback close");
// }

bool Amcl::checkUpdateThresholds()
{
  RCLCPP_DEBUG(this->get_logger(),"Checking for AMCL3D update");

  if (this->now() < nextupdate_time_)
    return false;

  odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

  /* Check translation threshold */
  if (odom_increment_tf_.getOrigin().length() > parameters_.d_th_)
  {
    RCLCPP_DEBUG(this->get_logger(),"Translation update");
    return true;
  }

  /* Check yaw threshold */
  double yaw, pitch, roll;
  odom_increment_tf_.getBasis().getRPY(roll, pitch, yaw);
  if (fabs(yaw) > parameters_.a_th_)
  {
    RCLCPP_DEBUG(this->get_logger(),"Rotation update");
    return true;
  }

  return false;
}

void Amcl::setInitialPose(const tf2::Transform& init_pose, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev)
{
  initodom_2_world_tf_ = init_pose;

  const tf2::Vector3 t = init_pose.getOrigin();

  const float x_init = t.x();
  const float y_init = t.y();
  const float z_init = t.z();
  const float a_init = static_cast<float>(getYawFromTf(init_pose));

  pf_.init(parameters_.num_particles_, x_init, y_init, z_init, a_init, x_dev, y_dev, z_dev, a_dev);

  mean_p_ = pf_.getMean();
  lastmean_p_ = mean_p_;

  /* Extract TFs for future updates */
  /* Reset lastupdatebase_2_odom_tf_ */
  lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

  /* Publish particles */
  publishParticles();
}

// double Node::getYawFromtf(const tf2::Transform & tf2)
// {
//   double yaw, pitch, roll;
//   tf.getBasis().getRPY(roll, pitch, yaw);

//   return yaw;
// }

double Amcl::getYawFromTf(const tf2::Transform& tf)
{
  double yaw, pitch, roll;
  tf.getBasis().getRPY(roll, pitch, yaw);

  return yaw;
}

void Amcl::rvizMarkerPublish(const uint32_t anchor_id, const float r, const geometry_msgs::msg::Point& uav,
                         const geometry_msgs::msg::Point& anchor)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = parameters_.global_frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "amcl3d";
  marker.id = anchor_id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = r;
  marker.scale.z = r;
  marker.color.a = 0.5;
  if (amcl_out_) /* Indicate if AMCL was lost */
  {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }
  else
  {
    switch (anchor_id)
    {
      case 1:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case 2:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      case 3:
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        break;
    }
  }
  marker.points.clear();
  marker.points.push_back(uav);
  marker.points.push_back(anchor);

  /* Publish marker */
  range_markers_pub_->publish(marker);
}

}  // namespace amcl3d

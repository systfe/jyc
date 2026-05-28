#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Pose3.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <mutex>

namespace robocon25_sim
{

class CmdVelModelPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    if (sdf->HasElement("cmd_timeout")) {
      cmd_timeout_ = sdf->Get<double>("cmd_timeout");
    }
    if (sdf->HasElement("publish_rate")) {
      publish_period_ = 1.0 / std::max(1.0, sdf->Get<double>("publish_rate"));
    }

    pose_ = model_->WorldPose();
    last_update_time_ = model_->GetWorld()->SimTime();
    last_cmd_time_ = last_update_time_;
    last_publish_time_ = last_update_time_;

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = *msg;
        last_cmd_time_ = model_->GetWorld()->SimTime();
      });

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&CmdVelModelPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    const auto now = model_->GetWorld()->SimTime();
    const double dt = (now - last_update_time_).Double();
    last_update_time_ = now;
    if (dt <= 0.0) {
      return;
    }

    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = cmd_;
      if ((now - last_cmd_time_).Double() > cmd_timeout_) {
        cmd = geometry_msgs::msg::Twist();
      }
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = pose_.Rot().Yaw();

    yaw += cmd.angular.z * dt;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double world_vx = cmd.linear.x * cos_yaw - cmd.linear.y * sin_yaw;
    const double world_vy = cmd.linear.x * sin_yaw + cmd.linear.y * cos_yaw;

    pose_.Pos().X() += world_vx * dt;
    pose_.Pos().Y() += world_vy * dt;
    pose_.Rot() = ignition::math::Quaterniond(roll, pitch, yaw);

    model_->SetWorldPose(pose_);
    model_->SetLinearVel(ignition::math::Vector3d(world_vx, world_vy, 0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, cmd.angular.z));

    if ((now - last_publish_time_).Double() >= publish_period_) {
      PublishOdom(now, cmd, world_vx, world_vy, yaw);
      last_publish_time_ = now;
    }
  }

  void PublishOdom(
    const gazebo::common::Time & stamp,
    const geometry_msgs::msg::Twist & cmd,
    double world_vx,
    double world_vy,
    double yaw)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = stamp.sec;
    odom.header.stamp.nanosec = stamp.nsec;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = pose_.Pos().X();
    odom.pose.pose.position.y = pose_.Pos().Y();
    odom.pose.pose.position.z = pose_.Pos().Z();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = cmd.linear.x;
    odom.twist.twist.linear.y = cmd.linear.y;
    odom.twist.twist.angular.z = cmd.angular.z;
    odom_pub_->publish(odom);
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist cmd_;
  ignition::math::Pose3d pose_;
  gazebo::common::Time last_update_time_;
  gazebo::common::Time last_cmd_time_;
  gazebo::common::Time last_publish_time_;
  double cmd_timeout_ = 0.25;
  double publish_period_ = 1.0 / 30.0;
};

GZ_REGISTER_MODEL_PLUGIN(CmdVelModelPlugin)

}  // namespace robocon25_sim

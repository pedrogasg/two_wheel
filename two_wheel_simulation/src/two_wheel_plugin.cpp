#include <two_wheel_simulation/two_wheel_plugin.hpp>

#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/PID.hh>
#include <sdf/sdf.hh>

#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <string>

namespace gazebo_plugins
{
  class TwoWheelPluginPrivate
  {
  private:
    gazebo_ros::Node::SharedPtr ros_node_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    gazebo::event::ConnectionPtr tick_;

    gazebo::physics::JointControllerPtr controller_;
    gazebo::physics::JointPtr left_joint_;
    gazebo::physics::JointPtr right_joint_;
    gazebo::physics::ModelPtr model_;

    sdf::ElementPtr sdf_;

    double wheel_radius_{10.0};

    double base_length_{10.0};

    double update_period_{0.016};
    
    /// Linear velocity in X received on command (m/s).
    double target_v_{0.0};

    /// Angular velocity in Z received on command (rad/s).
    double target_w_{0.0};

    std::mutex lock_;

  public:
    void OnLoad(const gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void CreateSubscription();
    void InitializeLeftJoint();
    void InitializeRightJoint();
    void SetLeftVelocity(double speed);
    void SetRightVelocity(double speed);
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
    void OnUpdate(const gazebo::common::UpdateInfo & _info);
  
  private:
    gazebo::physics::JointPtr InitializeJoint(std::string _joint_name);
  };

  void TwoWheelPluginPrivate::OnLoad(const gazebo::physics::ModelPtr _model,  sdf::ElementPtr _sdf)
  {
    model_ = _model;
    sdf_= _sdf;
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    controller_ = _model->GetJointController();
    wheel_radius_ = sdf_->GetElement("wheel_radius")->Get<double>();
    base_length_ = sdf_->GetElement("base_length")->Get<double>();
    auto update_rate = sdf_->GetElement("update_rate")->Get<double>();

    update_period_ = 1.0 / update_rate;
    
    RCLCPP_INFO(this->ros_node_->get_logger(),
     "\n The plugin is working in the model : [%s] \n",
     _model->GetScopedName());

  }

  void TwoWheelPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
  {


  }

  void TwoWheelPluginPrivate::CreateSubscription()
  {
    // To hte world

    tick_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&TwoWheelPluginPrivate::OnUpdate, this, std::placeholders::_1)
    );

    // To commands
    cmd_vel_subscription_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", ros_node_->get_qos().get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
      std::bind(&TwoWheelPluginPrivate::OnCmdVel, this, std::placeholders::_1)
    );

    RCLCPP_INFO(ros_node_->get_logger(),
     "\n Subscribed to [%s] \n",
      cmd_vel_subscription_->get_topic_name());
  }

  gazebo::physics::JointPtr TwoWheelPluginPrivate::InitializeJoint(std::string _joint_name)
  {
    auto joint_name = sdf_->GetElement(_joint_name)->Get<std::string>();
    auto joint = model_->GetJoint(joint_name);
    auto pid = gazebo::common::PID(0.1, 0, 0);
    controller_->SetVelocityPID(joint->GetScopedName(), pid);
    return joint;
  }

  void TwoWheelPluginPrivate::InitializeLeftJoint()
  {
    left_joint_ = InitializeJoint("left_joint");
  }

  void TwoWheelPluginPrivate::InitializeRightJoint()
  {
    right_joint_ = InitializeJoint("right_joint");
  }

  void TwoWheelPluginPrivate::SetLeftVelocity(double speed)
  {
    controller_->SetVelocityTarget(left_joint_->GetScopedName(), speed);
  }
  
  void TwoWheelPluginPrivate::SetRightVelocity(double speed)
  {
    controller_->SetVelocityTarget(right_joint_->GetScopedName(), speed);
  }

  void TwoWheelPluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    target_v_ = _msg->linear.x;
    target_w_ = _msg->angular.z;
    auto R_2 = wheel_radius_ * 2;
    auto V_2 = target_v_ * 2;
    auto L_w = base_length_ * target_w_;
    this->SetLeftVelocity((V_2 - L_w)/R_2);
    this->SetRightVelocity((V_2 + L_w)/R_2);
  }

  TwoWheelPlugin::TwoWheelPlugin(/* args */)
    : impl_(std::make_unique<TwoWheelPluginPrivate>())
  {
  } 

  TwoWheelPlugin::~TwoWheelPlugin()
  {
  }

  void TwoWheelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    impl_->OnLoad(_model, _sdf);
    impl_->InitializeLeftJoint();
    impl_->InitializeRightJoint();
    impl_->CreateSubscription();
  }

  GZ_REGISTER_MODEL_PLUGIN(TwoWheelPlugin)
  
} // namespace 


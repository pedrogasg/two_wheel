#include <two_wheel_simulation/two_wheel_plugin.hpp>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/PID.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <cstdio>
#include <string>

namespace gazebo_plugins
{
  class TwoWheelPluginPrivate
  {
  public:
    void OnLoad(const gazebo::physics::ModelPtr _model);
    gazebo_ros::Node::SharedPtr ros_node_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr left_joint_;
    gazebo::common::PID left_pid_;

  };

  void TwoWheelPluginPrivate::OnLoad(const gazebo::physics::ModelPtr _model)
  {
    std::cerr << "\nThe velodyne plugin is attach to model[" <<
        _model->GetName() << "]\n";
    RCLCPP_WARN(this->ros_node_->get_logger(), "\n The plugin is working in the model : \n", _model->GetName());
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
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
    impl_->model_ = _model;
    auto left_joint_name = _sdf->GetElement("left_joint")->Get<std::string>();
    impl_->left_joint_ = _model->GetJoint(left_joint_name);
    impl_->left_pid_ = gazebo::common::PID(0.1, 0, 0);
    auto controller = impl_->model_->GetJointController();
    controller->SetVelocityPID(impl_->left_joint_->GetScopedName(), impl_->left_pid_);
    controller->SetVelocityTarget(impl_->left_joint_->GetScopedName(), 10.0);
    impl_->OnLoad(_model);

    
  }

  GZ_REGISTER_MODEL_PLUGIN(TwoWheelPlugin)
  
} // namespace 


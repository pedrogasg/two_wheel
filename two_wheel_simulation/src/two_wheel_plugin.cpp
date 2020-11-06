#include <two_wheel_simulation/two_wheel_plugin.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdio>

namespace gazebo_plugins
{
  class TwoWheelPluginPrivate
  {
  public:
    void OnLoad(const gazebo::physics::ModelPtr _model);
    gazebo_ros::Node::SharedPtr ros_node_;
  };

  void TwoWheelPluginPrivate::OnLoad(const gazebo::physics::ModelPtr _model)
  {
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
    impl_->OnLoad(_model);
  }

  GZ_REGISTER_MODEL_PLUGIN(TwoWheelPlugin)
  
} // namespace 


#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo
{
  /// \brief A plugin for a contact sensor that publishes contact info to ROS 2.
  class ContactPlugin : public SensorPlugin
  {
  public:
    ContactPlugin();
    virtual ~ContactPlugin();

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  private:
    void OnUpdate();

    sensors::ContactSensorPtr parentSensor;
    event::ConnectionPtr updateConnection;

    // ROS 2 node and publisher
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_pub;
  };
}

#endif

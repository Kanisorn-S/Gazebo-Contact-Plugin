#include "ContactPlugin.hh"
#include <gazebo/msgs/contacts.pb.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
  // Initialize ROS 2 if not already done
  if (!rclcpp::ok())
  {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  // Create node and publisher
  this->ros_node = std::make_shared<rclcpp::Node>("contact_plugin_node");
  this->contact_pub =
      this->ros_node->create_publisher<std_msgs::msg::String>("/contact", 10);
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
  rclcpp::shutdown();
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  this->parentSensor =
      std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  msgs::Contacts contacts = this->parentSensor->Contacts();

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::string collision1 = contacts.contact(i).collision1();
    std::string collision2 = contacts.contact(i).collision2();

    // Extract model and link names from "model::link::collision"
    auto parse_name = [](const std::string &s) {
      std::vector<std::string> parts;
      std::stringstream ss(s);
      std::string item;
      while (std::getline(ss, item, ':'))
      {
        if (!item.empty())
          parts.push_back(item);
      }
      return parts;
    };

    auto p1 = parse_name(collision1);
    auto p2 = parse_name(collision2);

    if (p1.size() >= 2 && p2.size() >= 2)
    {
      std::string model1 = p1[0];
      std::string link1 = p1[1];
      std::string model2 = p2[0];
      std::string link2 = p2[1];

      std_msgs::msg::String msg;
      msg.data = "model1_name: " + model1 + ", link1_name: " + link1 +
                 ", model2_name: " + model2 + ", link2_name: " + link2;

      this->contact_pub->publish(msg);
    }
  }

  // Optional: spin node to process callbacks
  rclcpp::spin_some(this->ros_node);
}

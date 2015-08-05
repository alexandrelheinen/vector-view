#include "ContactManager.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactManager)

/////////////////////////////////////////////////
ContactManager::ContactManager() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactManager::~ContactManager()
{
  output_history->close();
}

/////////////////////////////////////////////////
void ContactManager::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  output_history = new std::ofstream("history_sensor.txt");
  this->parentSensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);  // Get the parent sensor.
  if (!this->parentSensor) // Make sure the parent sensor is valid.
  {
    gzerr << "ContactManager requires a ContactSensor.\n";
    return;
  }
  this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&ContactManager::OnUpdate, this)); // Connect to the sensor update event.
  this->parentSensor->SetActive(true); // Make sure the parent sensor is active.
}

/////////////////////////////////////////////////
void ContactManager::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts = this->parentSensor->GetContacts();
  unsigned int i, j;
  math::Vector3 force = math::Vector3::Zero;

  for (i = 0; i < contacts.contact_size(); ++i)
    for (j = 0; j < contacts.contact(i).wrench_size(); ++j)
      if (contacts.contact(i).wrench(j).body_1_name().find("iCub") != std::string::npos)
        force += msgs::Convert(
                                contacts.contact(i).wrench(j).body_1_wrench().force());

  if (output_history->is_open())
  {
    *output_history << contacts.time().sec() + 0.000000001*contacts.time().nsec() << " "
                  << force.GetLength()                                            << " "
                  << force.x                                                      << " "
                  << force.y                                                      << " "
                  << force.z                                                      << " "
                  << std::endl;
  } else
  {
    std::cout << "Unable to update de the file 'history.txt'" << std::endl;
  }
}

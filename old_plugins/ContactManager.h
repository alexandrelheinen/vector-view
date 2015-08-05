#ifndef _CONTACTMANAGER_
#define _CONTACTMANAGER_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{

  /// \brief An example plugin for a contact sensor.
  class ContactManager : public SensorPlugin
  {
  /// \brief Constructor.
  public:
    ContactManager();

  /// \brief Destructor.
    ~ContactManager();

  public:
  /// \brief Load the sensor plugin.
  /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  /// \brief Callback that recieves the contact sensor's update signal.
    void OnUpdate();

  private:
  /// \brief Pointer to the contact sensor
    sensors::ContactSensorPtr parentSensor;
  /// \brief Connection that maintains a link between the contact sensor's
  /// updated signal and the OnUpdate callback.
    event::ConnectionPtr updateConnection;
    std::vector<math::Vector3> data;
    std::ofstream *output_history;
  };
}
#endif

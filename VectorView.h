#ifndef VECTORVIEW_H
#define VECTORVIEW_H

#define FORCE_SCALE 2E-2 // scale between the forces intensities and the vectors length (unity N^-1)
#define NOISE_THRESHOLD 1E-6
#define ARROW_LENGTH .05
#define FILE_NAME "history.txt"

// Gazebo includes
#include <gazebo.hh>
#include <rendering/Visual.hh>
#include <rendering/rendering.hh>
#include <msgs/msgs.hh>
#include <common/common.hh>
// general includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


namespace gazebo
{
  typedef gazebo::msgs::Contacts* ContactsPtr;

  class VectorView : public VisualPlugin
  {
  public:
    // CONSTRUCTOR AND DESTRUCTOR
    VectorView();
    ~VectorView();
    // FUNCTIONS
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf); // executed once the plugin is loaded, just set this class as a sensor's subscriber
    void VectorViewUpdate(ConstContactsPtr &_msg);  // executed everytime a message is published by the sensor: updates the vector visual

  private:
    // FUNCTIONS
    std::string FindName();  // find the topic name and stores it at "topic_path"
    void PrintContact(math::Vector3 force);  // print the n-th data on the terminal
    void UpdateVector(math::Vector3 force);  // update the n-th vector based on the current contacts
    // VARIABLES
    rendering::VisualPtr visual;
    transport::SubscriberPtr subs;

    ContactsPtr contacts;  // current contacts
    rendering::DynamicLines* forceVector; // the vector representation line, a vector is used because the same sensor can have many contacts

    std::ofstream *output_history;
    std::string fileName;
  };
}

#endif // MY_PLUGIN_H

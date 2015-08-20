#ifndef VECTORVIEW_H
#define VECTORVIEW_H

#define FORCE_SCALE 2E-2 // scale between the forces intensities and the vectors length (unity N^-1)
#define NOISE_THRESHOLD 1E-6
#define ARROW_LENGTH .05

// Gazebo includes
#include <gazebo.hh>
#include <rendering/Visual.hh>
#include <rendering/rendering.hh>
#include <msgs/msgs.hh>
#include <common/common.hh>
// general includes
#include <iostream>
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
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf); // executed once the plugin is loaded
    void VectorViewUpdate(ConstContactsPtr &_msg);                 // executed everytime a message is published by the sensor: updates the vector visual

  private:
    // FUNCTIONS
    std::vector<std::string> FindName();     // find the topic, output history and collision names based on the visual
    void UpdateVector(math::Vector3 force);  // update visual from the vector
    // VARIABLES
    rendering::VisualPtr visual;
    transport::SubscriberPtr subs;

    ContactsPtr contacts;  // current contacts
    rendering::DynamicLines* forceVector; // the vector representation line, a vector is used because the same sensor can have many contacts

    std::ofstream *output_history;
    std::string conllisionName;
  };
}

#endif

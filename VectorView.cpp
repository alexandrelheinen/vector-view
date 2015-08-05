#include "VectorView.h"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VectorView)

VectorView::VectorView() {}

VectorView::~VectorView()
{
  output_history->close();
}

// find out the topic path string
// example of the visual name: robot::left_leg::vis_foot
// in this case we need 4 informations: robot, left, leg, foot (model, side, member and visual)
std::string VectorView::FindName()
{
  std::string name = this->visual->GetName();
  std::size_t u = name.find_last_of("_"); // u from underline
  std::size_t p = name.find_last_of(":"); // p from points
  std::string member_visual(name, u+1, name.length()); // = foot
  name.erase(p-1, name.length()); // we don't need "vis" // so we erase the last part, for instance we have robot::left_leg
  u = name.find_last_of("_");  // exactly the same
  std::string member(name, u+1, name.length()); // leg
  p = name.find_last_of(":");
  std::string side(name, p+1, u-p-1); // left
  p = name.find_first_of(":");
  std::string model(name, 0, p); // robot

  this->fileName = "history_"+model+"_"+side+"_"+member+".txt";

  // *** SOME CONFIRMATION PRINTS ***
  std::cout << "___ VISUAL PLUGIN LOADED ___"  << std::endl;
  std::cout << "Model:\t"    << model          << std::endl;
  std::cout << "  Side  :\t" << side           << std::endl;
  std::cout << "  Member:\t" << member         << std::endl;
  std::cout << "  Visual:\t" << member_visual  << std::endl;
  std::cout << "  File  :\t" << this->fileName << std::endl;
  std::cout << "____________________________"  << std::endl;
  // ********************************/

  return "/gazebo/default/" + model + "/iCub/" + side + "_" + member + "/" + side + "_" + member + "_contact";  // the topic path should be "/gazebo/default/robot/left_leg/left_leg_contact";
}

void VectorView::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  this->visual = _parent;
  this->visual->SetVisible(true);

  transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  this->subs = node->Subscribe(this->FindName(), &VectorView::VectorViewUpdate, this); // define this plugin as a listener of the sensor topic defined in topic_path
  this->forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  this->forceVector->setMaterial("Gazebo/Blue");
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
  for(int k = 0; k < 5; ++k) // -> needs five points
    this->forceVector->AddPoint(math::Vector3::Zero);

  output_history = new std::ofstream(fileName.c_str());
}

void VectorView::UpdateVector(math::Vector3 force)
{
  math::Vector3 begin = math::Vector3::Zero; // this->GetPosition(n);
  math::Vector3 end   = begin + FORCE_SCALE*force;
  // draw a cute arrow, just as a vector should be represented
  this->forceVector->SetPoint(0, begin);
  this->forceVector->SetPoint(1, end);
  this->forceVector->SetPoint(2, end - ARROW_LENGTH*math::Matrix3(1, 0, 0, 0, 0.9848, -0.1736, 0,  0.1736, 0.9848)*force.Normalize());
  this->forceVector->SetPoint(3, end);
  this->forceVector->SetPoint(4, end - ARROW_LENGTH*math::Matrix3(1, 0, 0, 0, 0.9848,  0.1736, 0, -0.1736, 0.9848)*force.Normalize());
}

// called when a new message is received
void VectorView::VectorViewUpdate(ConstContactsPtr &_msg)
{
  gazebo::msgs::Contacts c = *_msg;
  this->contacts = &c; // update contacts
  math::Vector3 force = math::Vector3::Zero;
  unsigned int n, m;

  /* ORIGINAL CODE (SOUNDS WORST)
  for(n = 0; n < this->contacts->contact_size(); ++n) // maybe we could change it for a for_each with a lamda function inside it
    for (m = 0; m < this->contacts->contact(n).wrench_size(); ++m)
      if (this->contacts->contact(n).wrench(m).body_1_name().find("iCub") != std::string::npos)
        force += msgs::Convert(
                                this->contacts->contact(n).wrench(m).body_1_wrench().force());
  */
  for(n = 0; n < this->contacts->contact_size(); ++n) // maybe we could change it for a for_each with a lamda function inside it
    for (m = 0; m < this->contacts->contact(n).wrench_size(); ++m)
      if (this->contacts->contact(n).wrench(m).body_1_name().find("iCub") != std::string::npos)
        force += msgs::Convert(this->contacts->contact(n).wrench(m).body_1_wrench().force());
      else if (this->contacts->contact(n).wrench(m).body_2_name().find("iCub") != std::string::npos)
        force -= msgs::Convert(this->contacts->contact(n).wrench(m).body_1_wrench().force());

  if(force.GetLength() < NOISE_THRESHOLD)
    force = math::Vector3::Zero;

  if (output_history->is_open())
    *output_history << contacts->time().sec() + 0.000000001*contacts->time().nsec() << " "
                    << force.GetLength()                                            << " "
                    << force.x                                                      << " "
                    << force.y                                                      << " "
                    << force.z                                                      << " "
                    << visual->GetWorldPose().pos.x                                  << " "
                    << std::endl;
  else
    std::cout << "Unable to update de the contact history file. ["<< this->fileName <<"];" << std::endl;

  this->UpdateVector(force);
}

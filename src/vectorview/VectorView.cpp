#include "vectorview/VectorView.h"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VectorView)

VectorView::VectorView() {}

VectorView::~VectorView()
{
  output_history->close();
}

void VectorView::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  fc = 2;
  // get visual and names
  this->visual = _parent;
  this->visual->SetVisible(true);
  std::vector<std::string> name = this->FindName();
  // define this plugin as a listener of the sensor topic defined in topic_path
  transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  this->subs = node->Subscribe(name.at(0), &VectorView::VectorViewUpdate, this);
  // visual components initialization
  this->forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  this->forceVector->setMaterial("Gazebo/Blue");
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
  for(int k = 0; k < 5; ++k) // -> needs five points
    this->forceVector->AddPoint(math::Vector3::Zero);
  // output history
  output_history = new std::ofstream(name.at(1).c_str());
  this->collisionName = name.at(2);
  // Filters setup
  Dsp::Params params;
  params[0] = 100;                 // sample rate
  params[1] = 5;                   // order
  params[2] = fc;                  // cutoff frequency
  this->filter = new Dsp::FilterDesign <Dsp::Butterworth::Design::LowPass <10>, 3>; // a 3 channel filter to a 3 dimention vector :)
  this->filter->setParams(params);
}

void VectorView::UpdateVector(math::Vector3 force)
{
  math::Quaternion orientation = visual->GetWorldPose().rot.GetInverse();
  math::Quaternion rotation    = visual->GetRotation();
  math::Vector3 begin          = math::Vector3::Zero;
  math::Vector3 end            = begin + FORCE_SCALE*(rotation.RotateVector(
                                                                            orientation.RotateVector(force)));
  // draw a cute arrow, just as a vector should be represented
  this->forceVector->SetPoint(0, begin);
  this->forceVector->SetPoint(1, end);
  this->forceVector->SetPoint(2, end - ARROW_LENGTH*math::Matrix3(1, 0, 0, 0, 0.9848, -0.1736, 0,  0.1736, 0.9848)*(end - begin).Normalize());
  this->forceVector->SetPoint(3, end);
  this->forceVector->SetPoint(4, end - ARROW_LENGTH*math::Matrix3(1, 0, 0, 0, 0.9848,  0.1736, 0, -0.1736, 0.9848)*(end - begin).Normalize());
}

// find out contact, output history file and collision names
std::vector<std::string> VectorView::FindName()
{
  std::vector<std::string> out(3);
  std::vector<std::string> names;
  std::string name = this->visual->GetName();

  while (name.find("::") != std::string::npos)
  {
    names.push_back(std::string(name, 0, name.find("::")));
    name.erase(0, name.find("::") + 2);
  }
  out.at(0) += "~";
  out.at(1) += "history";
  int i;
  for(i = 0; i < names.size(); ++i )
  {
    out.at(0) += "/" + names.at(i);
    out.at(1) += "_" + names.at(i);
    out.at(2) += "::" + names.at(i);
  }
  out.at(0) += "/" + names.at(i-1) + "_contact";
  out.at(1) += ".txt";
  out.at(2) += "::" + names.at(i-1) + "_collision";
  out.at(2).erase(0,2);

  // ***************** SOME CONFIRMATION PRINTS ****************
  std::cout << "______ VECTOR VIEW LOADED ______"  << std::endl;
  std::cout << "Robot:\t"       << names.at(0)     << std::endl;
  std::cout << "  Model    :\t" << names.at(1)     << std::endl;
  std::cout << "  Member   :\t" << names.at(2)    << std::endl;
  std::cout << "  Topic    :\t" << out.at(0)      << std::endl;
  std::cout << "  File     :\t" << out.at(1)      << std::endl;
  std::cout << "  Collision:\t" << out.at(2)      << std::endl;
  std::cout << "________________________________"  << std::endl;
  // ***********************************************************/
  return out; // contact topic // output file // collision name
}

// called when a new message is received
void VectorView::VectorViewUpdate(ConstContactsPtr &_msg)
{
  // update contacts
  msgs::Contacts c = *_msg;
  this->contacts = &c;
  math::Vector3 force = math::Vector3::Zero;

  // sum of all forces
  unsigned int n, m;
  for(n = 0; n < this->contacts->contact_size(); ++n)
  {
    for (m = 0; m < this->contacts->contact(n).wrench_size(); ++m)
    {
      if (this->contacts->contact(n).wrench(m).body_1_name().find(collisionName) != std::string::npos)
      {
        force += msgs::Convert(this->contacts->contact(n).wrench(m).body_1_wrench().force());
      } else if (this->contacts->contact(n).wrench(m).body_2_name().find(collisionName) != std::string::npos);
      {
        force -= msgs::Convert(this->contacts->contact(n).wrench(m).body_1_wrench().force());
      }
    }
  }

  // force filtering
  double length = force.GetLength();
  double* values[3];
  values[0] = &(force.x);
  values[1] = &(force.y);
  values[2] = &(force.z);
  filter->process(1, values);

  // write output_history */
  if (output_history->is_open())
    *output_history << contacts->time().sec() + 0.000000001*contacts->time().nsec() << " " // 1
                    << force.GetLength()                                            << " " // 2
                    << force.x                                                      << " " // 3
                    << force.y                                                      << " " // 4
                    << force.z                                                      << " " // 5
                    << length                                                       << " " // 6
                    << std::endl;
  else
    std::cout << "Unable to update de the contact history file. ["<< this->FindName().at(1) <<"];" << std::endl;

  // update visual DynamicLines
  if(force.GetLength() < NOISE_THRESHOLD)
    force = math::Vector3::Zero;
  this->UpdateVector(force);
}

#include "vectorview/VectorView.h"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VectorView)

VectorView::VectorView()
{
  std::cout << "VectorView created ..." << std::endl;
}

VectorView::~VectorView()
{
  output_history->close();
}


void VectorView::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  // get visual and names
  this->visual = _parent;
  this->visual->SetVisible(true);
  // Filters setup
  fc = 2;
  Dsp::Params params;
  params[0] = 100;                 // sample rate
  params[1] = 5;                   // order
  params[2] = fc;                  // cutoff frequency
  this->filter = new Dsp::FilterDesign <Dsp::Butterworth::Design::LowPass <10>, 3>; // a 3 channel filter to a 3 dimention vector :)
  this->filter->setParams(params);
  std::cout << "-- Plugin loaded" << std::endl;
}

void VectorView::Init()
{
  std::cout << "-- Init() begin" << std::endl;
  this->FindName();
  // define this plugin as a listener of the sensor topic defined in topic_path
  this->subs.reset();
  node.reset(new gazebo::transport::Node());
  node->Init();
  this->subs = node->Subscribe(topicName, &VectorView::VectorViewUpdate, this);
  // output history
  output_history.reset(new std::ofstream(fileName.c_str()));

  // visual components initialization
  this->forceVector.reset(this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP));
  this->forceVector->setMaterial("Gazebo/Blue");
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
  for(int k = 0; k < 5; ++k) // -> needs five points
    this->forceVector->AddPoint(math::Vector3::Zero);

  std::cout << "-- VectorView plugin initialized" << std::endl
            << "   topic path : " << topicName     << std::endl
            << "   output file: " << fileName      << std::endl
            << "   collsion   : " << collisionName << std::endl
            << std::endl;
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
void VectorView::FindName()
{
  std::vector<std::string> names;
  std::string name = this->visual->GetName();

  while (name.find("::") != std::string::npos)
  {
    names.push_back(std::string(name, 0, name.find("::")));
    name.erase(0, name.find("::") + 2);
  }
  topicName     = "~";
  fileName      = "history";
  collisionName = "";

  int i;
  for(i = 0; i < names.size(); ++i )
  {
    topicName     += "/" + names.at(i);
    fileName      += "_" + names.at(i);
    collisionName += "::" + names.at(i);
  }
  topicName     += "/" + names.at(i-1) + "_contact";
  fileName      += ".txt";
  collisionName += "::" + names.at(i-1) + "_collision";
  collisionName.erase(0,2);
}

// called when a new message is received
void VectorView::VectorViewUpdate(ConstContactsPtr &_msg)
{
  this->contacts.reset();
  this->contacts = ContactsPtr(_msg);
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
    std::cout << "Unable to update de the contact history file. ["<< this->fileName <<"];" << std::endl;

  // update visual DynamicLines
  if(force.GetLength() < NOISE_THRESHOLD)
    force = math::Vector3::Zero;
  this->UpdateVector(force);
}

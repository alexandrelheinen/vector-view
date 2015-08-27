#include "vectorview/VectorView.h"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VectorView)

VectorView::VectorView()
{
}

VectorView::~VectorView()
{
}


void VectorView::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  std::cout << "-- Load() begin";
  // get visual and names
  this->visual = _parent;
  // Filters setup
  std::cout << "  -- filter setup";
  fc = 2;
  Dsp::Params params;
  params[0] = RATE;                 // sample rate
  params[1] = 5;                   // order
  params[2] = fc;                  // cutoff frequency
  this->filter = new Dsp::FilterDesign <Dsp::Butterworth::Design::LowPass <10>, 3>; // a 3 channel filter to a 3 dimention vector :)
  this->filter->setParams(params);
  std::cout << "  -- Plugin loaded" << std::endl;
}

void VectorView::Init()
{
  std::cout << "-- Init() begin";
  this->FindName();

  std::cout << "  -- DynamicLines creation";
  // visual components initialization
  //this->forceVector.reset(this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP));
  this->forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  this->forceVector->setMaterial("Gazebo/Blue");
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
  for(int k = 0; k < 5; ++k) // -> needs five points
    this->forceVector->AddPoint(math::Vector3::Zero);

  std::cout << "  -- set as subscriber";
  // define this plugin as a listener of the sensor topic defined in topic_path
  this->subs.reset();
  node.reset(new gazebo::transport::Node());
  node->Init();
  this->subs = node->Subscribe(topicName, &VectorView::VectorViewUpdate, this);

  std::cout << std::endl;
  std::cout << "-- VectorView plugin initialized" << std::endl
            << "   topic path : " << topicName     << std::endl
            << "   collsion   : " << collisionName << std::endl
            << std::endl;
}

void VectorView::UpdateVector(math::Vector3 force)
{
  math::Quaternion orientation = visual->GetWorldPose().rot;
  math::Vector3 begin          = math::Vector3::Zero;
  math::Vector3 end            = begin + FORCE_SCALE*(orientation.RotateVectorReverse(force));
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
  collisionName = "";

  int i;
  for(i = 0; i < names.size(); ++i )
  {
    topicName     += "/" + names.at(i);
    collisionName += "::" + names.at(i);
  }
  topicName     += "/" + names.at(i-1) + "_contact";
  collisionName += "::" + names.at(i-1) + "_collision";
  collisionName.erase(0,2);
}

// called when a new message is received
void VectorView::VectorViewUpdate(ConstContactsPtr &_contactsMsg)
{
  math::Vector3 force = math::Vector3::Zero;

  // sum of all forces
  unsigned int n, m;
  for(n = 0; n < _contactsMsg->contact_size(); ++n)
  {
    for (m = 0; m < _contactsMsg->contact(n).wrench_size(); ++m)
    {
      if (_contactsMsg->contact(n).wrench(m).body_1_name().find(collisionName) != std::string::npos)
      {
        force = force + msgs::Convert(_contactsMsg->contact(n).wrench(m).body_1_wrench().force());
      } else if (_contactsMsg->contact(n).wrench(m).body_2_name().find(collisionName) != std::string::npos);
      {
        force = force - msgs::Convert(_contactsMsg->contact(n).wrench(m).body_1_wrench().force());
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

  // update visual DynamicLines
  if(force.GetLength() < NOISE_THRESHOLD)
    force = math::Vector3::Zero;
  this->UpdateVector(force);
}

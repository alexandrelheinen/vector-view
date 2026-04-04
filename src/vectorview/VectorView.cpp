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
  this->visual = _parent;
  filter = std::make_unique<Dsp::ForceFilter>();
}

void VectorView::Init()
{
  this->FindName();

  this->forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->forceVector->setMaterial("Gazebo/Blue");
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
  for (int k = 0; k < 6; ++k)  // three lines → six points
    this->forceVector->AddPoint(ignition::math::Vector3d::Zero);

  this->forceVector->Update();

  this->subs.reset();
  node.reset(new gazebo::transport::Node());
  node->Init();
  this->subs = node->Subscribe(topicName, &VectorView::VectorViewUpdate, this);
  std::cout << std::endl
            << "-- VectorView plugin initialized" << std::endl
            << "   topic path : " << topicName     << std::endl
            << "   collision  : " << collisionName << std::endl
            << std::endl;
}

void VectorView::UpdateVector(ignition::math::Vector3d force)
{
  ignition::math::Vector3d begin = ignition::math::Vector3d::Zero;
  ignition::math::Vector3d end   = begin + FORCE_SCALE *
    (visual->WorldPose().Rot().RotateVectorReverse(force));

  this->forceVector->SetPoint(0, begin);
  this->forceVector->SetPoint(1, end);
  this->forceVector->SetPoint(2, end);
  this->forceVector->SetPoint(3, end - ARROW_LENGTH *
    ignition::math::Matrix3d(1, 0, 0, 0, 0.9848, -0.1736, 0,  0.1736, 0.9848) *
    (end - begin).Normalized());
  this->forceVector->SetPoint(4, end);
  this->forceVector->SetPoint(5, end - ARROW_LENGTH *
    ignition::math::Matrix3d(1, 0, 0, 0, 0.9848,  0.1736, 0, -0.1736, 0.9848) *
    (end - begin).Normalized());
}

void VectorView::FindName()
{
  std::string visualName = this->visual->GetName();
  if (!VectorView::ParseVisualName(visualName, topicName, collisionName))
  {
    gzerr << "[VectorView] Could not parse visual name '" << visualName << "'.\n";
  }
}

void VectorView::VectorViewUpdate(ConstContactsPtr& message)
{
  ignition::math::Vector3d force = ignition::math::Vector3d::Zero;

  int n, m;
  for (n = 0; n < message->contact_size(); ++n)
  {
    for (m = 0; m < message->contact(n).wrench_size(); ++m)
    {
      if (message->contact(n).wrench(m).body_1_name().find(collisionName) != std::string::npos)
        force += msgs::Convert(message->contact(n).wrench(m).body_1_wrench().force());
      else
        force += msgs::Convert(message->contact(n).wrench(m).body_2_wrench().force());
    }
  }

  if (message->contact_size())
    force = force / n;

  filter->Filter(&force);

  if (force.Length() < NOISE_THRESHOLD)
    force = ignition::math::Vector3d::Zero;

  this->UpdateVector(force);
}

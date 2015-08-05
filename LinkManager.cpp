#include "LinkManager.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(LinkManager)

/////////////////////////////////////////////////
LinkManager::LinkManager() : ModelPlugin()
{
}

/////////////////////////////////////////////////
LinkManager::~LinkManager()
{
  output_history->close();
}

/////////////////////////////////////////////////
void LinkManager::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model = _model;
  output_history = new std::ofstream("history_link.txt");
  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LinkManager::OnUpdate, this));
  r_hand = model->GetLink("r_hand");
  l_hand = model->GetLink("l_hand");
}

/////////////////////////////////////////////////
void LinkManager::OnUpdate()
{
  math::Vector3 r_vel = r_hand->GetWorldAngularVel();
  math::Vector3 r_acc = r_hand->GetWorldAngularAccel();
  math::Vector3 r_pos = r_hand->GetWorldPose().pos;
  math::Vector3 l_vel = l_hand->GetWorldAngularVel();
  math::Vector3 l_acc = l_hand->GetWorldAngularAccel();
  math::Vector3 l_pos = l_hand->GetWorldPose().pos;

  if (output_history->is_open())
  {
    *output_history << physics::get_world("default")->GetSimTime().Double() << " "
                    << r_pos.GetLength()                                    << " "
                    << l_pos.GetLength()                                    << " "
                    << r_vel.GetLength()                                    << " "
                    << l_vel.GetLength()                                    << " "
                    << r_acc.GetLength()                                    << " "
                    << l_acc.GetLength()                                    << " "
                    << std::endl;
  } else
  {
    std::cout << "Unable to update de the file 'history_link.txt'" << std::endl;
  }
}

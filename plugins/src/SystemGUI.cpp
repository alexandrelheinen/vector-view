#include "plugins/include/SystemGUI.h"
using namespace gazebo;
GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)

SystemGUI::~SystemGUI()
{
  this->connections.clear();
}

void SystemGUI::Load(int _argc, char** _argv)
{
  this->connections.push_back(event::Events::ConnectPreRender(boost::bind(&SystemGUI::Update, this)));
  this->app       = new QApplication(_argc, _argv);
  this->interface = new Interface();
  this->interface->show();
  this->app->exec();
}

void SystemGUI::Init()
{

}

void SystemGUI::Update()
{

}

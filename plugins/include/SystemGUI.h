#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include "Interface.h"

namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
  public:
    ~SystemGUI();
    void Load(int _argc, char** _argv);
    void Init();

  private:
    void Update();
    std::vector<event::ConnectionPtr> connections;
    Interface* interface;
    QApplication* app;
  };
}

#include "vectorview/Constants.h"
#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"
#include <gazebo.hh>

using namespace gazebo;
namespace Dsp
{
  class ForceFilter
  {
  public:
    ForceFilter();
    ~ForceFilter();
    double Filter(math::Vector3* vector);

  private:
    Dsp::Filter* filter;
  };
}

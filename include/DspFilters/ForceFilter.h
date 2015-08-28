#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"
#include <gazebo.hh>

#define RATE 25

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

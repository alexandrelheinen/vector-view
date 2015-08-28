#include "DspFilters/ForceFilter.h"
using namespace Dsp;
using namespace gazebo;

ForceFilter::ForceFilter()
{
  Dsp::Params params;
  params[0] = RATE;                 // sample rate
  params[1] = 3;                   // order
  params[2] = 1.5;             // cutoff frequency
  this->filter = new Dsp::FilterDesign <Dsp::Butterworth::Design::LowPass <10>, 3>; // a 3 channel filter to a 3 dimention vector :)
  this->filter->setParams(params);
}

double ForceFilter::Filter(math::Vector3* force)
{
  double length = force->GetLength();
  double* values[3];
  values[0] = &(force->x);
  values[1] = &(force->y);
  values[2] = &(force->z);
  filter->process(1, values);
  return length;
}

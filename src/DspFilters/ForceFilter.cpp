#include "DspFilters/ForceFilter.h"
using namespace Dsp;

ForceFilter::ForceFilter()
{
  Dsp::Params params;
  params[0] = RATE;   // sample rate
  params[1] = 3;      // order
  params[2] = 1.5;    // cutoff frequency
  this->filter = new Dsp::FilterDesign<Dsp::Butterworth::Design::LowPass<10>, 3>;
  this->filter->setParams(params);
}

ForceFilter::~ForceFilter()
{
  delete filter;
}

double ForceFilter::Filter(ignition::math::Vector3d* force)
{
  double length = force->Length();
  // Extract components into temporaries so that we can pass mutable pointers
  // to the DSP library, then write the filtered values back.
  double x = force->X(), y = force->Y(), z = force->Z();
  double* values[3] = { &x, &y, &z };
  filter->process(1, values);
  force->Set(x, y, z);
  return length;
}

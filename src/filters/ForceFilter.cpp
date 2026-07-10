#include "vectorview/ForceFilter.h"

using namespace vectorview;

ForceFilter::ForceFilter() {
  Dsp::Params params;
  params[0] = RATE;
  params[1] = 3;
  params[2] = 1.5;
  filter.reset(new Dsp::FilterDesign<Dsp::Butterworth::Design::LowPass<10>, 3>);
  filter->setParams(params);
}

ForceFilter::~ForceFilter() {}

double ForceFilter::Filter(Vec3* force) {
  const double length = force->Length();
  double* values[3];
  values[0] = &(force->x);
  values[1] = &(force->y);
  values[2] = &(force->z);
  filter->process(1, values);
  return length;
}

#include "vectorview/ForceFilter.h"

using namespace vectorview;

ForceFilter::ForceFilter() {
  Init(RATE, 3, 1.5);
}

ForceFilter::ForceFilter(double sample_rate, int order, double cutoff_hz) {
  Init(sample_rate, order, cutoff_hz);
}

ForceFilter::~ForceFilter() {}

void ForceFilter::Configure(double sample_rate, int order, double cutoff_hz) {
  Init(sample_rate, order, cutoff_hz);
}

void ForceFilter::Init(double sample_rate, int order, double cutoff_hz) {
  Dsp::Params params;
  params[0] = sample_rate;
  params[1] = order;
  params[2] = cutoff_hz;
  filter.reset(new Dsp::FilterDesign<Dsp::Butterworth::Design::LowPass<10>, 3>);
  filter->setParams(params);
}

double ForceFilter::Filter(Vec3* force) {
  const double length = force->Length();
  double* values[3];
  values[0] = &(force->x);
  values[1] = &(force->y);
  values[2] = &(force->z);
  filter->process(1, values);
  return length;
}

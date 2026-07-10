#ifndef VECTORVIEW_FORCE_FILTER_H
#define VECTORVIEW_FORCE_FILTER_H

#include "vectorview/Constants.h"
#include "vectorview/ContactUtils.h"

#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"

#include <memory>

namespace vectorview {

class ForceFilter {
 public:
  ForceFilter();
  explicit ForceFilter(double sample_rate, int order, double cutoff_hz);
  ~ForceFilter();

  double Filter(Vec3* vector);
  void Configure(double sample_rate, int order, double cutoff_hz);

 private:
  void Init(double sample_rate, int order, double cutoff_hz);

  std::unique_ptr<Dsp::Filter> filter;
};

}  // namespace vectorview

#endif

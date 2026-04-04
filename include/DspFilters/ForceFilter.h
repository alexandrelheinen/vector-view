#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"
#include <ignition/math/Vector3.hh>

#define RATE 25

namespace Dsp
{
  /**
   * @brief 3-axis Butterworth low-pass filter for contact force vectors.
   *
   * Wraps a Dsp::Filter with a 3-channel design (one channel per spatial axis)
   * and exposes a single Filter() call that operates directly on a
   * ignition::math::Vector3d in place.
   */
  class ForceFilter
  {
  public:
    /** @brief Constructs the filter with order 3 and 1.5 Hz cut-off at 25 Hz. */
    ForceFilter();
    /** @brief Destroys the filter, freeing the inner Dsp::Filter object. */
    ~ForceFilter();

    /**
     * @brief Applies the low-pass filter to a force vector in place.
     * @param vector Pointer to the force vector to be filtered.
     * @return The pre-filter magnitude of the vector (N).
     */
    double Filter(ignition::math::Vector3d* vector);

  private:
    Dsp::Filter* filter;
  };
}

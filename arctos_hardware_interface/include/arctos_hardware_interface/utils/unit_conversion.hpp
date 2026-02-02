#ifndef ARCTOS_HARDWARE_INTERFACE__UTILS__UNIT_CONVERSION_HPP_
#define ARCTOS_HARDWARE_INTERFACE__UTILS__UNIT_CONVERSION_HPP_

#include <cstddef>
#include <vector>

namespace arctos_hardware_interface
{
namespace utils
{

class UnitConverter
{
public:
  UnitConverter(
    const std::vector<double>& steps_per_revolution,
    const std::vector<bool>& joint_inversions);

  double rad_to_steps(double radians, size_t joint_index) const;
  double steps_to_rad(double steps, size_t joint_index) const;
  double rad_per_sec_to_steps_per_sec(double rad_per_sec, size_t joint_index) const;
  double steps_per_sec_to_rad_per_sec(double steps_per_sec, size_t joint_index) const;

private:
  const std::vector<double>& steps_per_revolution_;
  const std::vector<bool>& joint_inversions_;
  static constexpr double TWO_PI = 6.283185307179586;
};

}  // namespace utils
}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE__UTILS__UNIT_CONVERSION_HPP_

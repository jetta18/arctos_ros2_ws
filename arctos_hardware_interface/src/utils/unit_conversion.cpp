#include "arctos_hardware_interface/utils/unit_conversion.hpp"

namespace arctos_hardware_interface
{
namespace utils
{

UnitConverter::UnitConverter(
  const std::vector<double>& steps_per_revolution,
  const std::vector<bool>& joint_inversions)
: steps_per_revolution_(steps_per_revolution),
  joint_inversions_(joint_inversions)
{
}

double UnitConverter::rad_to_steps(double radians, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  double effective_radians = joint_inversions_[joint_index] ? -radians : radians;
  return effective_radians * (steps_per_revolution_[joint_index] / TWO_PI);
}

double UnitConverter::steps_to_rad(double steps, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  double radians = steps * (TWO_PI / steps_per_revolution_[joint_index]);
  return joint_inversions_[joint_index] ? -radians : radians;
}

double UnitConverter::rad_per_sec_to_steps_per_sec(double rad_per_sec, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  double effective_rad_per_sec = joint_inversions_[joint_index] ? -rad_per_sec : rad_per_sec;
  return effective_rad_per_sec * (steps_per_revolution_[joint_index] / TWO_PI);
}

double UnitConverter::steps_per_sec_to_rad_per_sec(double steps_per_sec, size_t joint_index) const
{
  if (joint_index >= steps_per_revolution_.size())
  {
    return 0.0;
  }
  
  double rad_per_sec = steps_per_sec * (TWO_PI / steps_per_revolution_[joint_index]);
  return joint_inversions_[joint_index] ? -rad_per_sec : rad_per_sec;
}

}  // namespace utils
}  // namespace arctos_hardware_interface

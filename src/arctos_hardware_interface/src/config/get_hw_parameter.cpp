#include "arctos_hardware_interface/arctos_mks_hardware_interface.hpp"
#include <string>

namespace arctos_hardware_interface
{

std::string ArctosMKSHardwareInterface::getHardwareParameter(
  const std::string& param_name, const std::string& default_value) const
{
  auto it = info_.hardware_parameters.find(param_name);
  if (it != info_.hardware_parameters.end()) {
    return it->second;
  }
  return default_value;
}

} // namespace arctos_hardware_interface

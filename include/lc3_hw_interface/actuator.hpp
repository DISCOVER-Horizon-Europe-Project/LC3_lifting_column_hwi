#ifndef LC3_HW_INTERFACE__ACTUATOR_HPP
#define LC3_HW_INTERFACE__ACTUATOR_HPP

#include <string>
#include <cmath>

namespace lc3_hw_interface
{
  class Actuator
  {
  public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;

    Actuator() = default;

    explicit Actuator(const std::string &Actuator_name)
    {
      setup(Actuator_name);
    }

    void setup(const std::string &Actuator_name)
    {
      name = Actuator_name;
    }
  };

} // namespace lc3_hw_interface

#endif // LC3_HW_INTERFACE__ACTUATOR_HPP

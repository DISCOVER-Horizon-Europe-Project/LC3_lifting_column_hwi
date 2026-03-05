#ifndef LC3_HW_INTERFACE__WHEEL_HPP
#define LC3_HW_INTERFACE__WHEEL_HPP

#include <string>
#include <cmath>

namespace lc3_hw_interface
{
  class Wheel
  {
  public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;

    Wheel() = default;

    explicit Wheel(const std::string &wheel_name)
    {
      setup(wheel_name);
    }

    void setup(const std::string &wheel_name)
    {
      name = wheel_name;
    }
  };

} // namespace lc3_hw_interface

#endif // LC3_HW_INTERFACE__WHEEL_HPP

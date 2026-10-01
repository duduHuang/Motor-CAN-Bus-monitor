// include/motor/domain/torque_types.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

enum class TorqueLocation : uint8_t {
    Unknown = 0,
    MotorShaft,
    OutputShaft
};

[[nodiscard]] constexpr bool is_valid_torque_location(TorqueLocation loc) noexcept {
    return loc == TorqueLocation::MotorShaft || loc == TorqueLocation::OutputShaft;
}

} // namespace motor::domain
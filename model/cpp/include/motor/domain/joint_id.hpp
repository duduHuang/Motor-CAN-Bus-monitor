// include/motor/domain/joint_id.hpp
#pragma once

#include <cstdint>

namespace motor::domain {

enum class JointId : uint8_t {
    Unknown = 0,
    RightFrontHipX, RightFrontHipY, RightFrontKnee,
    LeftFrontHipX,  LeftFrontHipY,  LeftFrontKnee,
    RightRearHipX,  RightRearHipY,  RightRearKnee,
    LeftRearHipX,   LeftRearHipY,   LeftRearKnee,
    Count
};

[[nodiscard]] constexpr bool is_valid_joint_id(JointId id) noexcept {
    return id > JointId::Unknown && id < JointId::Count;
}

} // namespace motor::domain
#pragma once

#include <stdint.h>

namespace Time {
uint32_t Micros();

inline float get_dt_seconds(uint32_t previous_us, uint32_t current_us) {
    if (previous_us == 0U)
        return 0.0f;
    return static_cast<float>(current_us - previous_us) * 0.000001f;
}
} // namespace Time

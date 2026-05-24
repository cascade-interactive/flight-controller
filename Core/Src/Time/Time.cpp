#include "Time.hpp"
#include "main.h"

extern TIM_HandleTypeDef htim6;

namespace Time {

uint32_t Micros() {
    uint32_t ms = HAL_GetTick();
    uint32_t us = __HAL_TIM_GET_COUNTER(&htim6);

    return (ms * 1000U) + us;
}

} // namespace Time
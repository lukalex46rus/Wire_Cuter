#include "DRV8825.h"

// Определение статической константы в .cpp файле
const uint8_t DRV8825::MS_TABLE[6] = {0b000, 0b001, 0b010, 0b011, 0b100, 0b111};

DRV8825::DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin)
: A4988(steps, dir_pin, step_pin)
{
}

DRV8825::DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin)
: A4988(steps, dir_pin, step_pin, enable_pin)
{
}

DRV8825::DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin, 
                 int16_t mode0_pin, int16_t mode1_pin, int16_t mode2_pin)
: A4988(steps, dir_pin, step_pin, mode0_pin, mode1_pin, mode2_pin)
{
}

DRV8825::DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin,
                 int16_t mode0_pin, int16_t mode1_pin, int16_t mode2_pin)
: A4988(steps, dir_pin, step_pin, enable_pin, mode0_pin, mode1_pin, mode2_pin)
{
}
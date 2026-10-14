#include "A4988.h"

// Определение статической константы в .cpp файле
const uint8_t A4988::MS_TABLE[5] = {0b000, 0b001, 0b010, 0b011, 0b111};

A4988::A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin)
: BasicStepperDriver(steps, dir_pin, step_pin),
  ms1_pin(PIN_UNCONNECTED),
  ms2_pin(PIN_UNCONNECTED), 
  ms3_pin(PIN_UNCONNECTED)
{
}

A4988::A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin)
: BasicStepperDriver(steps, dir_pin, step_pin, enable_pin),
  ms1_pin(PIN_UNCONNECTED),
  ms2_pin(PIN_UNCONNECTED),
  ms3_pin(PIN_UNCONNECTED)
{
}

A4988::A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin, 
             int16_t ms1_pin, int16_t ms2_pin, int16_t ms3_pin)
: BasicStepperDriver(steps, dir_pin, step_pin),
  ms1_pin(ms1_pin),
  ms2_pin(ms2_pin),
  ms3_pin(ms3_pin)
{
}

A4988::A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin,
             int16_t ms1_pin, int16_t ms2_pin, int16_t ms3_pin)
: BasicStepperDriver(steps, dir_pin, step_pin, enable_pin),
  ms1_pin(ms1_pin),
  ms2_pin(ms2_pin),
  ms3_pin(ms3_pin)
{
}

void A4988::begin(float rpm, uint16_t microsteps) {
    BasicStepperDriver::begin(rpm, microsteps);

    if (!IS_CONNECTED(ms1_pin) || !IS_CONNECTED(ms2_pin) || !IS_CONNECTED(ms3_pin)) {
        return;
    }

    pinMode(ms1_pin, OUTPUT);
    pinMode(ms2_pin, OUTPUT);
    pinMode(ms3_pin, OUTPUT);
    
    setMicrostep(microsteps);
}

uint16_t A4988::setMicrostep(uint16_t microsteps) {
    BasicStepperDriver::setMicrostep(microsteps);

    if (!IS_CONNECTED(ms1_pin) || !IS_CONNECTED(ms2_pin) || !IS_CONNECTED(ms3_pin)) {
        return this->microsteps;
    }

    const uint8_t* ms_table = getMicrostepTable();
    size_t ms_table_size = getMicrostepTableSize();

    for (uint16_t i = 0; i < ms_table_size; i++) {
        if (this->microsteps == (1 << i)) {
            uint8_t mask = ms_table[i];
            digitalWrite(ms3_pin, (mask & 4) ? HIGH : LOW);
            digitalWrite(ms2_pin, (mask & 2) ? HIGH : LOW);
            digitalWrite(ms1_pin, (mask & 1) ? HIGH : LOW);
            break;
        }
    }
    
    return this->microsteps;
}
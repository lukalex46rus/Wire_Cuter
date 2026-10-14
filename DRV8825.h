#ifndef DRV8825_H
#define DRV8825_H
#include <Arduino.h>
#include "A4988.h"

class DRV8825 : public A4988 {
private:
    // Объявление статической константы
    static const uint8_t MS_TABLE[6];
    static constexpr uint16_t MAX_MICROSTEP = 32;
    
    static constexpr int16_t step_high_min = 2;
    static constexpr int16_t step_low_min = 2;
    static constexpr int16_t wakeup_time = 1700;

protected:
    virtual const uint8_t* getMicrostepTable() const override { return MS_TABLE; }
    virtual size_t getMicrostepTableSize() const override { return sizeof(MS_TABLE); }
    virtual uint16_t getMaxMicrostep() const override { return MAX_MICROSTEP; }

public:
    DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin);
    DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin);
    DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin, 
            int16_t mode0_pin, int16_t mode1_pin, int16_t mode2_pin);
    DRV8825(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin,
            int16_t mode0_pin, int16_t mode1_pin, int16_t mode2_pin);
};

#endif
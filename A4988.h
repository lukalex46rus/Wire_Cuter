#ifndef A4988_H
#define A4988_H
#include <Arduino.h>
#include "BasicStepperDriver.h"

class A4988 : public BasicStepperDriver {
private:
    // Объявление статической константы
    static const uint8_t MS_TABLE[5];
    static constexpr uint16_t MAX_MICROSTEP = 16;
    
    const int16_t ms1_pin;
    const int16_t ms2_pin;
    const int16_t ms3_pin;
    
    static constexpr int16_t step_high_min = 1;
    static constexpr int16_t step_low_min = 1;
    static constexpr int16_t wakeup_time = 1000;

protected:
    virtual const uint8_t* getMicrostepTable() const { return MS_TABLE; }
    virtual size_t getMicrostepTableSize() const { return sizeof(MS_TABLE); }
    virtual uint16_t getMaxMicrostep() const override { return MAX_MICROSTEP; }

public:
    A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin);
    A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin);
    A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin, 
          int16_t ms1_pin, int16_t ms2_pin, int16_t ms3_pin);
    A4988(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin,
          int16_t ms1_pin, int16_t ms2_pin, int16_t ms3_pin);
    
    void begin(float rpm = 60.0f, uint16_t microsteps = 1) override;
    uint16_t setMicrostep(uint16_t microsteps) override;
};

#endif
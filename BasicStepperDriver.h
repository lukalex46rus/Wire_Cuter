#ifndef STEPPER_DRIVER_BASE_H
#define STEPPER_DRIVER_BASE_H
#include <Arduino.h>

constexpr int16_t PIN_UNCONNECTED = -1;
constexpr bool IS_CONNECTED(int16_t pin) { return pin != PIN_UNCONNECTED; }

constexpr uint32_t STEP_PULSE(uint16_t steps, uint16_t microsteps, float rpm) {
    return static_cast<uint32_t>(60.0f * 1000000L / steps / microsteps / rpm);
}

constexpr uint32_t MIN_YIELD_MICROS = 50;

class BasicStepperDriver {
public:
    enum class Mode : uint8_t { CONSTANT_SPEED, LINEAR_SPEED };
    enum class State : uint8_t { STOPPED, ACCELERATING, CRUISING, DECELERATING };
    
    struct Profile {
        Mode mode = Mode::CONSTANT_SPEED;
        int16_t accel = 1000;
        int16_t decel = 1000;
    };
    
    template<typename T>
    static constexpr const T& stepperMin(const T& a, const T& b) {
        return (b < a) ? b : a;
    }
    
    template<typename T>
    static constexpr const T& stepperMax(const T& a, const T& b) {
        return (a < b) ? b : a;
    }

    static inline void delayMicros(uint32_t delay_us, uint32_t start_us = 0) {
        if (delay_us == 0) return;
        
        if (start_us == 0) {
            start_us = micros();
        }
        
        if (delay_us > MIN_YIELD_MICROS) {
            yield();
        }
        
        while (micros() - start_us < delay_us) {
        }
    }

private:
    int32_t rest;
    uint32_t last_action_end;
    uint32_t next_action_interval;

protected:
    const uint16_t motor_steps;
    const int16_t dir_pin;
    const int16_t step_pin;
    const int16_t enable_pin;
    
    int16_t enable_active_state;
    uint16_t microsteps;
    
    static constexpr int16_t step_high_min = 1;
    static constexpr int16_t step_low_min = 1;
    static constexpr int16_t wakeup_time = 0;
    static constexpr uint16_t MAX_MICROSTEP = 128;
    
    float rpm;
    Profile profile;
    
    int32_t step_count;
    int32_t steps_remaining;
    int32_t steps_to_cruise;
    int32_t steps_to_brake;
    int32_t step_pulse;
    int32_t cruise_step_pulse;
    int16_t dir_state;
    
    int32_t paused_steps_remaining;
    int32_t paused_step_count;
    int32_t paused_step_pulse;
    int32_t paused_cruise_step_pulse;
    int32_t paused_steps_to_cruise;
    int32_t paused_steps_to_brake;
    int16_t paused_dir_state;
    bool is_paused;

    void calcStepPulse(void);
    void alterMove(int32_t steps);

public:
    BasicStepperDriver(uint16_t steps, int16_t dir_pin, int16_t step_pin);
    BasicStepperDriver(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin);
    
    virtual ~BasicStepperDriver() = default;
    
    virtual void begin(float rpm = 60.0f, uint16_t microsteps = 1);
    virtual uint16_t setMicrostep(uint16_t microsteps);
    virtual uint16_t getMaxMicrostep() const { return MAX_MICROSTEP; }
    
    uint16_t getMicrostep() const { return microsteps; }
    uint16_t getSteps() const { return motor_steps; }
    float getRPM() const { return rpm; }
    Profile getSpeedProfile() const { return profile; }
    
    void setRPM(float rpm);
    void setSpeedProfile(Mode mode, int16_t accel = 1000, int16_t decel = 1000);
    void setSpeedProfile(const Profile& new_profile) { profile = new_profile; }
    
    void move(int32_t steps);
    void rotate(int32_t deg);
    void rotate(double deg);
    
    void setEnableActiveState(int16_t state) { enable_active_state = state; }
    virtual void enable();
    virtual void disable();
    
    void pause();
    void resume();
    
    void startMove(int32_t steps, int32_t time = 0);
    void startRotate(int32_t deg);
    void startRotate(double deg);
    int32_t nextAction();
    
    void startBrake();
    int32_t stop();
    
    State getCurrentState() const;
    int32_t getStepsCompleted() const { return step_count; }
    int32_t getStepsRemaining() const { return steps_remaining; }
    int16_t getDirection() const { return (dir_state == HIGH) ? 1 : -1; }
    
    int32_t getTimeForMove(int32_t steps) const;
    
    template<typename T>
    int32_t calcStepsForRotation(T deg) const {
        return static_cast<int32_t>(deg * motor_steps * microsteps / 360);
    }
};

#endif
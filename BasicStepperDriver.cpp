#include "BasicStepperDriver.h"

BasicStepperDriver::BasicStepperDriver(uint16_t steps, int16_t dir_pin, int16_t step_pin)
: BasicStepperDriver(steps, dir_pin, step_pin, PIN_UNCONNECTED)
{
}

BasicStepperDriver::BasicStepperDriver(uint16_t steps, int16_t dir_pin, int16_t step_pin, int16_t enable_pin)
: motor_steps(steps), 
  dir_pin(dir_pin), 
  step_pin(step_pin), 
  enable_pin(enable_pin),
  enable_active_state(HIGH),
  microsteps(1),
  rpm(0),
  rest(0),
  last_action_end(0),
  next_action_interval(0),
  step_count(0),
  steps_remaining(0),
  dir_state(0),
  steps_to_cruise(0),
  steps_to_brake(0),
  step_pulse(0),
  cruise_step_pulse(0),
  is_paused(false)
{
    profile.mode = Mode::CONSTANT_SPEED;
    profile.accel = 1000;
    profile.decel = 1000;
}

void BasicStepperDriver::begin(float rpm, uint16_t microsteps) {
    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, HIGH);

    pinMode(step_pin, OUTPUT);
    digitalWrite(step_pin, LOW);

    if (IS_CONNECTED(enable_pin)) {
        pinMode(enable_pin, OUTPUT);
        disable();
    }

    this->rpm = rpm;
    setMicrostep(microsteps);
    enable();
}

uint16_t BasicStepperDriver::setMicrostep(uint16_t microsteps) {
    for (uint16_t ms = 1; ms <= getMaxMicrostep(); ms <<= 1) {
        if (microsteps == ms) {
            this->microsteps = microsteps;
            break;
        }
    }
    return this->microsteps;
}

void BasicStepperDriver::setSpeedProfile(Mode mode, int16_t accel, int16_t decel) {
    profile.mode = mode;
    profile.accel = accel;
    profile.decel = decel;
}

void BasicStepperDriver::move(int32_t steps) {
    startMove(steps);
    while (nextAction() > 0) {
    }
}

void BasicStepperDriver::rotate(int32_t deg) {
    move(calcStepsForRotation(deg));
}

void BasicStepperDriver::rotate(double deg) {
    move(calcStepsForRotation(deg));
}

void BasicStepperDriver::startMove(int32_t steps, int32_t time) {
    float speed;
    
    dir_state = (steps >= 0) ? HIGH : LOW;
    last_action_end = 0;
    steps_remaining = labs(steps);
    step_count = 0;
    rest = 0;
    
    switch (profile.mode) {
    case Mode::LINEAR_SPEED:
        speed = rpm * motor_steps / 60.0f;
        if (time > 0) {
            float t = time / 1e6f;
            float d = steps_remaining / static_cast<float>(microsteps);
            float a2 = 1.0f / profile.accel + 1.0f / profile.decel;
            float sqrt_candidate = t * t - 2 * a2 * d;
            
            if (sqrt_candidate >= 0) {
                // Исправление: приведение типов для stepperMin
                float calculated_speed = (t - sqrt(sqrt_candidate)) / a2;
                speed = stepperMin(speed, calculated_speed);
            }
        }
        
        steps_to_cruise = static_cast<int32_t>(microsteps * (speed * speed / (2 * profile.accel)));
        steps_to_brake = static_cast<int32_t>(steps_to_cruise * profile.accel / profile.decel);
        
        if (steps_remaining < steps_to_cruise + steps_to_brake) {
            steps_to_cruise = static_cast<int32_t>(steps_remaining * profile.decel / 
                                                 (profile.accel + profile.decel));
            steps_to_brake = steps_remaining - steps_to_cruise;
        }
        
        step_pulse = static_cast<int32_t>(1e6f * 0.676f * sqrt(2.0f / profile.accel / microsteps));
        cruise_step_pulse = static_cast<int32_t>(1e6f / speed / microsteps);
        break;

    case Mode::CONSTANT_SPEED:
    default:
        steps_to_cruise = 0;
        steps_to_brake = 0;
        step_pulse = cruise_step_pulse = STEP_PULSE(motor_steps, microsteps, rpm);
        
        if (time > steps_remaining * step_pulse) {
            step_pulse = static_cast<int32_t>(time / steps_remaining);
        }
    }
}

void BasicStepperDriver::alterMove(int32_t steps) {
    switch (getCurrentState()) {
    case State::ACCELERATING:
    case State::CRUISING:
        if (steps >= 0) {
            steps_remaining += steps;
        } else {
            steps_remaining = stepperMax(steps_to_brake, steps_remaining + steps);
        }
        break;
        
    case State::DECELERATING:
        break;
        
    case State::STOPPED:
        startMove(steps);
        break;
    }
}

void BasicStepperDriver::startBrake(void) {
    switch (getCurrentState()) {
    case State::CRUISING:
        steps_remaining = steps_to_brake;
        break;

    case State::ACCELERATING:
        steps_remaining = static_cast<int32_t>(step_count * profile.accel / profile.decel);
        break;

    default:
        break;
    }
}

int32_t BasicStepperDriver::stop(void) {
    int32_t retval = steps_remaining;
    steps_remaining = 0;
    is_paused = false;
    return retval;
}

int32_t BasicStepperDriver::getTimeForMove(int32_t steps) const {
    float t;
    int32_t cruise_steps;
    float speed;
    
    if (steps == 0) {
        return 0;
    }
    
    switch (profile.mode) {
        case Mode::LINEAR_SPEED:
            cruise_steps = steps - steps_to_cruise - steps_to_brake;
            speed = rpm * motor_steps / 60.0f;
            t = (cruise_steps / (microsteps * speed)) +
                sqrt(2.0f * steps_to_cruise / profile.accel / microsteps) +
                sqrt(2.0f * steps_to_brake / profile.decel / microsteps);
            t *= 1e6f;
            break;
            
        case Mode::CONSTANT_SPEED:
        default:
            t = steps * STEP_PULSE(motor_steps, microsteps, rpm);
    }
    
    return static_cast<int32_t>(t + 0.5f);
}

void BasicStepperDriver::startRotate(int32_t deg) {
    startMove(calcStepsForRotation(deg));
}

void BasicStepperDriver::startRotate(double deg) {
    startMove(calcStepsForRotation(deg));
}

void BasicStepperDriver::calcStepPulse(void) {
    if (steps_remaining <= 0) {
        return;
    }
    
    steps_remaining--;
    step_count++;

    if (profile.mode == Mode::LINEAR_SPEED) {
        switch (getCurrentState()) {
        case State::ACCELERATING:
            if (step_count < steps_to_cruise) {
                step_pulse = step_pulse - (2 * step_pulse + rest) / (4 * step_count + 1);
                rest = (step_count < steps_to_cruise) ? (2 * step_pulse + rest) % (4 * step_count + 1) : 0;
            } else {
                step_pulse = cruise_step_pulse;
            }
            break;

        case State::DECELERATING:
            step_pulse = step_pulse - (2 * step_pulse + rest) / (-4 * steps_remaining + 1);
            rest = (2 * step_pulse + rest) % (-4 * steps_remaining + 1);
            break;

        default:
            break;
        }
    }
}

int32_t BasicStepperDriver::nextAction(void) {
    if (is_paused) {
        return 0;
    }
    
    if (steps_remaining > 0) {
        delayMicros(next_action_interval, last_action_end);
        
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
        
        uint32_t m = micros();
        uint32_t pulse = step_pulse;
        calcStepPulse();
        
        delayMicros(step_high_min);
        digitalWrite(step_pin, LOW);
        
        last_action_end = micros();
        m = last_action_end - m;
        next_action_interval = (pulse > m) ? pulse - m : 1;
    } else {
        last_action_end = 0;
        next_action_interval = 0;
    }
    
    return next_action_interval;
}

BasicStepperDriver::State BasicStepperDriver::getCurrentState(void) const {
    if (steps_remaining <= 0) {
        return State::STOPPED;
    } else if (steps_remaining <= steps_to_brake) {
        return State::DECELERATING;
    } else if (step_count <= steps_to_cruise) {
        return State::ACCELERATING;
    } else {
        return State::CRUISING;
    }
}

void BasicStepperDriver::enable(void) {
    if (IS_CONNECTED(enable_pin)) {
        digitalWrite(enable_pin, enable_active_state);
    }
    delayMicros(2);
}

void BasicStepperDriver::disable(void) {
    if (IS_CONNECTED(enable_pin)) {
        digitalWrite(enable_pin, (enable_active_state == HIGH) ? LOW : HIGH);
    }
}

void BasicStepperDriver::pause(void) {
    if (steps_remaining > 0 && !is_paused) {
        paused_steps_remaining = steps_remaining;
        paused_step_count = step_count;
        paused_step_pulse = step_pulse;
        paused_cruise_step_pulse = cruise_step_pulse;
        paused_steps_to_cruise = steps_to_cruise;
        paused_steps_to_brake = steps_to_brake;
        paused_dir_state = dir_state;
        
        steps_remaining = 0;
        is_paused = true;
        
        digitalWrite(step_pin, LOW);
    }
}

void BasicStepperDriver::resume(void) {
    if (is_paused) {
        steps_remaining = paused_steps_remaining;
        step_count = paused_step_count;
        step_pulse = paused_step_pulse;
        cruise_step_pulse = paused_cruise_step_pulse;
        steps_to_cruise = paused_steps_to_cruise;
        steps_to_brake = paused_steps_to_brake;
        dir_state = paused_dir_state;
        
        last_action_end = 0;
        next_action_interval = 0;
        rest = 0;
        
        is_paused = false;
        digitalWrite(dir_pin, dir_state);
        delayMicros(2);
    }
}
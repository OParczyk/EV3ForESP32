#include "EV3RegulatedMotor.h"

const static char *TAG = "EV3RegulatedMotor";
char EV3RegulatedMotor::used_mcpwm_pairs=0;

void EV3RegulatedMotor::start()
{
#ifdef DRV8833
    mcpwm_io_signals_t temp[2];
    char channel = get_mcpwm_motor_channels(temp);
    if (channel==255){
        ESP_LOGE(TAG, "No free PWM channel available!");
    }
    ESP_LOGD(TAG, "Assigned MCPWM channel %u",channel);
    MOTOR_A=temp[0];
    MOTOR_B=temp[1];
    TIMER=get_timer(channel);
    if (TIMER==MCPWM_TIMER_MAX){
        ESP_LOGE(TAG, "No free MCPWM Timer available!");
    }

    // Setup motor control config with 50khz pwm freq
    mcpwm_config = {
    .frequency = 50000,
    .cmpr_a = 0,
    .cmpr_b = 0,
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER,
    };
#endif

#ifndef DRV8833
    pinMode(_motorPin1, OUTPUT);
    pinMode(_motorPin2, OUTPUT);
#endif

#ifdef DRV8833
    mcpwm_gpio_init(drv8833_mcpwm_unit, MOTOR_A, _motorPin1);
    mcpwm_gpio_init(drv8833_mcpwm_unit, MOTOR_B, _motorPin2);
#endif

#ifndef DRV8833
    // Setup channel 0 with 5khz pwm freq with 8 bit resolution.
    ledcSetup(0, 5000, 8);
#endif

#ifdef DRV8833
    mcpwm_init(drv8833_mcpwm_unit, TIMER, &mcpwm_config);
#endif

    // Set motors to coast mode
#ifndef DRV8833
    digitalWrite(_motorPin1, LOW);
    digitalWrite(_motorPin2, LOW);
#else
    mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_A, 0);
    mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_B, 0);
#endif

    // Setup motor encoder
    encoder.attachFullQuad(_tachoPin1, _tachoPin2);

    if (_motorCtrlHandle)
    {
        vTaskDelete(_motorCtrlHandle);
    }

    xTaskCreatePinnedToCore(
        &motorCtrlHelper,
        "EV3 motor",
        10000,
        this,
        1,
        &_motorCtrlHandle, // Task handle
        1
    );
}

void EV3RegulatedMotor::setSpeed(int32_t speed)
{
    //ESP_LOGE(TAG, "PID speed control not yet implemented!");
    //ESP_LOGD(TAG, "Set motor target speed to %d", speed);
    set(EV3RegulationType::SPEED, speed, KP_SPEED_CTRL, KI_SPEED_CTRL, KD_SPEED_CTRL);
}

void EV3RegulatedMotor::setPosition(int64_t position)
{
    ESP_LOGD(TAG, "Set motor target position to %d ticks", position);
    set(EV3RegulationType::POSITION, position, KP_POS_CTRL, KI_POS_CTRL, KD_POS_CTRL);
}

void EV3RegulatedMotor::setPositionSpeed(int64_t position, int32_t speed){
    _max_speed=abs(speed);
    set(EV3RegulationType::POSITION_AND_SPEED, position, KP_SPEED_CTRL, KI_SPEED_CTRL, KD_SPEED_CTRL);
}

void EV3RegulatedMotor::brake()
{
    ESP_LOGD(TAG, "Set motor to brake mode");
    xSemaphoreTake(_pidMutex, portMAX_DELAY);
    _reg_type = EV3RegulationType::NONE;
    xSemaphoreGive(_pidMutex);
#ifndef DRV8833
    switch (_motorState)
    {
    case EV3MotorState::BRAKE:
        break;                 // Already breaking
    case EV3MotorState::COAST: // Move from coasting to brake
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, HIGH);
        break;
    case EV3MotorState::FORWARD:
        ledcDetachPin(_motorPin1);
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, HIGH);
        break;
    case EV3MotorState::REVERSE:
        ledcDetachPin(_motorPin2);
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, HIGH);
        break;
    }
#else
    mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_A, 100);
    mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_B, 100);
#endif
    _motorState = EV3MotorState::BRAKE;
}

void EV3RegulatedMotor::coast()
{
    ESP_LOGD(TAG, "Set motor to coast mode");
    xSemaphoreTake(_pidMutex, portMAX_DELAY);
    _reg_type = EV3RegulationType::NONE;
    xSemaphoreGive(_pidMutex);
#ifndef DRV8833
    switch (_motorState)
    {
    case EV3MotorState::COAST:
        break;                 // Already coasting
    case EV3MotorState::BRAKE: // Move from coasting to brake
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);
        break;
    case EV3MotorState::FORWARD:
        ledcDetachPin(_motorPin1);
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);
        break;
    case EV3MotorState::REVERSE:
        ledcDetachPin(_motorPin2);
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);
        break;
    }
#else
    mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_A, 0);
    mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_B, 0);
#endif
    _motorState = EV3MotorState::COAST;
}

void EV3RegulatedMotor::setPWM(boolean direction, uint8_t value)
{
    if (direction)
    {
#ifndef DRV8833
        if (_motorState == EV3MotorState::REVERSE)
        {
            ledcDetachPin(_motorPin2);
        }
        if (_motorState != EV3MotorState::FORWARD)
        {
            ledcAttachPin(_motorPin1, 0);
            digitalWrite(_motorPin2, LOW);
            _motorState = EV3MotorState::FORWARD;
        }
#else
        mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_A, 0);
        mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_B, value/255.0*100.0);
        _motorState = EV3MotorState::FORWARD;
#endif
    }
    else
    {
#ifndef DRV8833
        if (_motorState == EV3MotorState::FORWARD)
        {
            ledcDetachPin(_motorPin1);
        }
        if (_motorState != EV3MotorState::REVERSE)
        {
            ledcAttachPin(_motorPin2, 0);
            digitalWrite(_motorPin1, LOW);
            _motorState = EV3MotorState::REVERSE;
        }
#else
        mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_A, value/255.0*100.0);
        mcpwm_set_duty(drv8833_mcpwm_unit, TIMER, MCPWM_GEN_B, 0);
        _motorState == EV3MotorState::REVERSE;
#endif

    }
#ifndef DRV8833
    ledcWrite(0, value);
#endif
}

void EV3RegulatedMotor::set(EV3RegulationType type, int64_t value, double Kp, double Ki, double Kd)
{
    xSemaphoreTake(_pidMutex, portMAX_DELAY);
    _reg_type = type;
    _reg_target = value;
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _outputSumPos = 0.0;
    _outputSumSpeed = 0.0;
    xSemaphoreGive(_pidMutex);
}

int32_t EV3RegulatedMotor::getSpeed()
{
    xSemaphoreTake(_speedMutex, portMAX_DELAY);
    auto speed = _speed;
    xSemaphoreGive(_speedMutex);
    return speed;
}

int64_t EV3RegulatedMotor::getPosition()
{
    xSemaphoreTake(_positionMutex, portMAX_DELAY);
    auto pos = encoder.getCount();
    xSemaphoreGive(_positionMutex);
    return pos;
}

void EV3RegulatedMotor::stop()
{
    ESP_LOGD(TAG, "Motor control started");
    if (_motorCtrlHandle)
    {
        vTaskDelete(_motorCtrlHandle);
        _motorCtrlHandle = nullptr;
    }
}

void EV3RegulatedMotor::motorCtrl()
{
    for (;;)
    {
        // Fetch position
        xSemaphoreTake(_positionMutex, portMAX_DELAY);
        auto pos = encoder.getCount();
        xSemaphoreGive(_positionMutex);

        auto time = millis();
        auto delta_pos = pos - _prev_pos;
        auto delta_time = time - _prev_time;

        // 720 ticks per round and 60*1000 ms per minute
        int32_t speed = ((delta_pos) * 1000 * 60) / (720 * delta_time);

        // Update speed
        xSemaphoreTake(_speedMutex, portMAX_DELAY);
        _speed = speed;
        xSemaphoreGive(_speedMutex);

        // Perform pid control if necessary.
        xSemaphoreTake(_pidMutex, portMAX_DELAY);
        if (_reg_type == EV3RegulationType::POSITION)
        {
            double output = 0;
            double dInput = delta_pos;
            double error = (double(_reg_target)) - ((double)pos);

            _outputSumPos += (_Ki * error);
            if (_outputSumPos > PWM_MAX)
            {
                _outputSumPos = PWM_MAX;
            }
            else if (_outputSumPos < PWM_MIN)
            {
                _outputSumPos = PWM_MIN;
            }

            output += _Kp * error;

            output += _outputSumPos - _Kd * dInput;

            if (output > PWM_MAX)
            {
                output = PWM_MAX;
            }
            else if (output < PWM_MIN)
            {
                output = PWM_MIN;
            }

            setPWM(output > 0, abs(output));
        }
        else if (_reg_type == EV3RegulationType::SPEED)
        {
            double output = 0;
            double dInput = speed - _prev_speed;
            double error = (double(_reg_target)) - ((double)speed);

            _outputSumSpeed += (_Ki * error);
            if (_outputSumSpeed > PWM_MAX)
            {
                _outputSumSpeed = PWM_MAX;
            }
            else if (_outputSumSpeed < PWM_MIN)
            {
                _outputSumSpeed = PWM_MIN;
            }

            output += _Kp * error;

            output += _outputSumSpeed - _Kd * dInput;

            if (output > PWM_MAX)
            {
                output = PWM_MAX;
            }
            else if (output < PWM_MIN)
            {
                output = PWM_MIN;
            }

            // TODO calculate pwm from output
            setPWM(output > 0, abs(output));
        }
        else if (_reg_type == EV3RegulationType::POSITION_AND_SPEED)
        {
            //calculate target speed. This should be fine as PWM was a standin for speed anyway.

            double output_speed = 0;
            double dInput_pos = delta_pos;
            double error_pos = (double(_reg_target)) - ((double)pos);

            _outputSumPos += (KI_POS_CTRL * error_pos);
            if (_outputSumPos > _max_speed)
            {
                _outputSumPos = _max_speed;
            }
            else if (_outputSumPos < -_max_speed)
            {
                _outputSumPos = -_max_speed;
            }

            output_speed += KP_POS_CTRL * error_pos;

            output_speed += _outputSumPos - KD_POS_CTRL * dInput_pos;

            if (output_speed > _max_speed)
            {
                output_speed = _max_speed;
            }
            else if (output_speed < -_max_speed)
            {
                output_speed = -_max_speed;
            }

            if (abs(error_pos)<=1)
                output_speed=0;

            //Then calculate PWM to reach speed

            double output_pwm = 0;
            double dInput_speed = speed - _prev_speed;
            double error_speed = output_speed - ((double)speed);

            _outputSumSpeed += (KI_SPEED_CTRL * error_speed);
            if (_outputSumSpeed > PWM_MAX)
            {
                _outputSumSpeed = PWM_MAX;
            }
            else if (_outputSumSpeed < PWM_MIN)
            {
                _outputSumSpeed = PWM_MIN;
            }

            output_pwm += KP_SPEED_CTRL * error_speed;

            output_pwm += _outputSumSpeed - KD_SPEED_CTRL * dInput_speed;

            if (output_pwm > PWM_MAX)
            {
                output_pwm = PWM_MAX;
            }
            else if (output_pwm < PWM_MIN)
            {
                output_pwm = PWM_MIN;
            }

            // TODO calculate pwm from output
            setPWM(output_pwm > 0, abs(output_pwm));
        }
        xSemaphoreGive(_pidMutex);

        _prev_pos = pos;
        _prev_time = time;

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
#include "StrokeEngine.h"

#include <Arduino.h>

#include "pattern.h"

const char* SE = "STROKE ENGINE";

// static pointer to engine and _servo
void StrokeEngine::begin(machineProperties *machine, FastAccelStepper *servo) {
    _servo = servo;
    _machine = machine;

    // Derived Machine Geometry & Motor Limits in steps:
    _travel = _machine->physicalTravel;
    _minStep = 0;
    _maxStep = int(0.5 + _travel * _machine->stepsPerMillimeter);
    _maxStepPerSecond = int(0.5 + _machine->maxSpeed * _machine->stepsPerMillimeter);
    _maxStepAcceleration = int(0.5 + _machine->maxAcceleration * _machine->stepsPerMillimeter);
    _index = 0;
    _depth = _maxStep;
    _previousDepth = _maxStep;
    _stroke = _maxStep / 3;
    _previousStroke = _maxStep / 3;
    _speed = 0;
    _sensation = 0.0;

    _state = READY;

    ESP_LOGI(SE, "Servo Initialized");
}

void StrokeEngine::setSpeed(float speed, bool applyNow = false) {
    // Update pattern with new speed, will be used with the next stroke or on
    // update request
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        //Convert percentage speed to steps per second
        _speed = (speed/100.0) * (_machine->maxSpeed * _machine->stepsPerMillimeter);
        pattern->setSpeed(_speed);

        ESP_LOGD(SE, "Speed: %f", _speed);

        // When running a pattern and immediate update requested:
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

            ESP_LOGD(SE, "Apply new settings now");
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }
}

void StrokeEngine::setDepth(float depth, bool applyNow = false) {
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        // Convert depth from mm into steps
        // Constrain depth between minStep and maxStep
        _depth = constrain(int(depth * _machine->stepsPerMillimeter), _minStep, _maxStep);

        pattern->setDepth(_depth);

        ESP_LOGD(SE, "Depht: %f", _depth);

        // When running a pattern and immediate update requested:
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

            ESP_LOGD(SE, "Apply new settings now");
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }
}

void StrokeEngine::setStroke(float stroke, bool applyNow = false) {
    // Update pattern with new stroke, will be used with the next stroke or on
    // update request
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        // Convert stroke from mm into steps
        // Constrain stroke between minStep and maxStep
        _stroke = constrain(int(stroke * _machine->stepsPerMillimeter), _minStep, _maxStep);

        pattern->setStroke(_stroke);

        ESP_LOGD(SE, "Stroke: %f", _stroke);

        // When running a pattern and immediate update requested:
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

            ESP_LOGD(SE, "Apply new settings now");
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }
}

void StrokeEngine::setSensation(float sensation, bool applyNow = false) {
    // Update pattern with new sensation, will be used with the next stroke or
    // on update request
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        // Constrain sensation between -100 and 100
        _sensation = constrain(sensation, -100, 100);

        pattern->setSensation(_sensation);

        ESP_LOGD(SE, "Sensation: %f", _sensation);

        // When running a pattern and immediate update requested:
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

            ESP_LOGD(SE, "Apply new settings now");
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }
}

bool StrokeEngine::setPattern(StrokePatterns NextPattern, bool applyNow = false) {
    // Reject invalid pattern indices and retain the previous pattern.
    if ((NextPattern < StrokePatterns::SimpleStroke) ||
        (NextPattern >= StrokePatterns::Count)) {
        ESP_LOGE(SE, "Invalid pattern index: %d", (int)NextPattern);
        return false;
    }

    // Build the replacement before letting go of the current one, so the engine
    // is never left without a valid pattern.
    Pattern *nextPattern = Pattern::Create(NextPattern);
    if (nextPattern == NULL) {
        ESP_LOGE(SE, "Out of memory creating pattern: %d", (int)NextPattern);
        return false;
    }

    // The stroking task dereferences `pattern` while holding _patternMutex, so
    // the pointer may only be swapped under that same mutex. Deleting and
    // re-creating it unlocked is a use-after-free: the stroking task can read
    // members of the freed object, or dispatch through a stale vtable pointer
    // into the freshly allocated replacement.
    Pattern *previousPattern = NULL;
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        previousPattern = pattern;
        pattern = nextPattern;

        // Inject current motion parameters into new pattern
        pattern->setSpeedLimit(_maxStepPerSecond, _maxStepAcceleration, _machine->stepsPerMillimeter);
        pattern->setStroke(_stroke);
        pattern->setDepth(_depth);
        pattern->setSensation(_sensation);
        pattern->setSpeed(_speed);

        ESP_LOGD(SE, "Pattern: %d", NextPattern);

        // When running a pattern and immediate update requested:
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

            ESP_LOGD(SE, "Apply new settings now");
        }

        // Reset index counter
        _index = -1;

        // give back mutex
        xSemaphoreGive(_patternMutex);
    } else {
        // The replacement was never published, so drop it and keep the current
        // pattern.
        delete nextPattern;
        return false;
    }

    // Free the previous pattern outside the mutex. It is no longer reachable by
    // any other task once swapped above, so this cannot race with the stroking
    // task, and it keeps the critical section short.
    delete previousPattern;

    return true;
}

bool StrokeEngine::startPattern() {
    // Only valid if state is ready
    if (_state == READY) {
        // Stop current move, should one be pending
        if (_servo->isRunning()) {
            // Stop _servo motor as fast as legally allowed
            _servo->setAcceleration(_maxStepAcceleration);
            _servo->applySpeedAcceleration();
            _servo->stopMove();
        }

        // Set state to PATTERN
        _state = PATTERN;

        // Reset Stroke and Motion parameters
        _index = -1;
        if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
            pattern->setSpeedLimit(_maxStepPerSecond, _maxStepAcceleration, _machine->stepsPerMillimeter);
            pattern->setStroke(_stroke);
            pattern->setDepth(_depth);
            pattern->setSensation(_sensation);
            pattern->setSpeed(_speed);
            xSemaphoreGive(_patternMutex);
        }

        if (_taskStrokingHandle == NULL) {
            // Create Stroke Task
            xTaskCreatePinnedToCore(
                this->_strokingImpl,   // Function that should be called
                "Stroking",            // Name of the task (for debugging)
                4096,                  // Stack size (bytes)
                this,                  // Pass reference to this class instance
                24,                    // Pretty high task priority
                &_taskStrokingHandle,  // Task handle
                1                      // Pin to application core
            );
        } else {
            // Resume task, if it already exists
            vTaskResume(_taskStrokingHandle);
        }

        ESP_LOGD(SE,"Started motion task.");

        return true;

    } else {
        ESP_LOGD(SE,"Failed to start motion");
        return false;
    }
}

void StrokeEngine::stopMotion() {
    // only valid when
    if (_state == PATTERN) {
        // Set state
        _state = READY;

        // Stop _servo motor as fast as legally allowed
        _servo->setAcceleration(_maxStepAcceleration);
        _servo->applySpeedAcceleration();
        _servo->stopMove();

        ESP_LOGD(SE,"Motion stopped");

        // Wait for _servo stopped
        while (_servo->isRunning())
        ;

        // Send telemetry data
        if (_callbackTelemetry != NULL) {
            _callbackTelemetry(float(_servo->getCurrentPosition() / _machine->stepsPerMillimeter), 0.0, false);
        }
    }
}

ServoState StrokeEngine::getState() { return _state; }

void StrokeEngine::registerTelemetryCallback(void (*callbackTelemetry)(float, float, bool)) {
    _callbackTelemetry = callbackTelemetry;
}

void StrokeEngine::_stroking() {
    motionParameter currentMotion;

    while (1) {  // infinite loop
        // Suspend task, if not in PATTERN state
        if (_state != PATTERN) {
            vTaskSuspend(_taskStrokingHandle);
        }

        // Take mutex to ensure no interference / race condition with
        // communication threat on other core
        if (xSemaphoreTake(_patternMutex, 0) == pdTRUE) {
            if (_applyUpdate == true) {
                // Ask pattern for update on motion parameters
                currentMotion = pattern->nextTarget(_index);

                // Increase deceleration if required to avoid crash
                if (_servo->getAcceleration() > currentMotion.acceleration) {
                    ESP_LOGD(SE,"Crash avoidance! Set acceleration from %f to %f",currentMotion.acceleration,_servo->getAcceleration());
                    currentMotion.acceleration = _servo->getAcceleration();
                }

                // Apply new trapezoidal motion profile to _servo
                _applyMotionProfile(&currentMotion);

                // clear update flag
                _applyUpdate = false;
            }

            // If motor has stopped issue moveTo command to next position
            else if (_servo->isRunning() == false) {
                // Increment index for pattern
                _index++;

                // Querey new set of pattern parameters
                currentMotion = pattern->nextTarget(_index);

                // Pattern may introduce pauses between strokes
                if (currentMotion.skip == false) {
                    // Apply new trapezoidal motion profile to _servo
                    _applyMotionProfile(&currentMotion);

                } else {
                    // decrement _index so that it stays the same until the next
                    // valid stroke parameters are delivered
                    _index--;
                }
            }

            // give back mutex
            xSemaphoreGive(_patternMutex);
        }

        // Delay 10ms
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void StrokeEngine::_applyMotionProfile(motionParameter *motion) {
    bool clipping = false;
    float speed = 0.0;
    float position = 0.0;

    // Apply new trapezoidal motion profile to _servo if pattern does not skip
    if (motion->skip == false) {
        // Constrain speed to below _maxStepPerSecond
        if (motion->speed > _maxStepPerSecond) {
            ESP_LOGD(SE,"Constrain speed: %f -> %f mm/s", motion->speed, _maxStepPerSecond);
            motion->speed = _maxStepPerSecond;
            clipping = true;
        }

        // Constrain acceleration between 1 step/sec^2 and _maxStepAcceleration
        if (motion->acceleration > _maxStepAcceleration) {
            ESP_LOGD(SE,"Constrain acceleration: %f -> %f mm/s²", motion->acceleration, _maxStepAcceleration);
            motion->acceleration = _maxStepAcceleration;
            clipping = true;
        }

        // Constrain stroke to motion envelope
        int pos = constrain((motion->stroke), _minStep, _maxStep);

        // write values to _servo
        _servo->setSpeedInHz(motion->speed);
        _servo->setAcceleration(motion->acceleration);
        _servo->moveTo(pos);

        // Compile speed telemetry data
        speed = float(motion->speed / _machine->stepsPerMillimeter);
        position = float(pos / _machine->stepsPerMillimeter);


        // Send telemetry data
        if (_callbackTelemetry != NULL) {
            _callbackTelemetry(position, speed, clipping);
        }
    }
}
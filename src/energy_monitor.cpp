#include "energy_monitor.h"

#include <cmath>
#include <esp_timer.h>
#include <esp_log.h>

RTC_DATA_ATTR EnergyMonitor::MeasureAccumulators EnergyMonitor::_accumulators = {};
RTC_DATA_ATTR EnergyMonitor::MeasureAccumulators EnergyMonitor::_cycleAccumulators = {};

EnergyMonitor::EnergyMonitor(int pinSda, int pinScl, TwoWire& wire)
    : _wire(wire),
      _pinSda(pinSda),
      _pinScl(pinScl) {}

EnergyMonitor::~EnergyMonitor() {
    this->stop();
}

void EnergyMonitor::stop() {
    if (_measureTaskHandle != nullptr) {
        if (_measureMutex != nullptr) {
            xSemaphoreTake(_measureMutex, portMAX_DELAY);
        }
        vTaskDelete(_measureTaskHandle);
        _measureTaskHandle = nullptr;
        if (_measureMutex != nullptr) {
            xSemaphoreGive(_measureMutex);
        }
    }
    if (_measureMutex != nullptr) {
        vSemaphoreDelete(_measureMutex);
        _measureMutex = nullptr;
    }
}

bool EnergyMonitor::begin() {
    if (_beginCalled) {
        return true;
    }

    if (!_wire.begin(_pinSda, _pinScl, I2cFrequency)) {
        ESP_LOGE(LogTag, "Failed to start I2C bus: SDA=%d SCL=%d", _pinSda, _pinScl);
        return false;
    }

    if (!this->beginSensor()) {
        return false;
    }

    if (_measureMutex == nullptr) {
        _measureMutex = xSemaphoreCreateMutex();
    }

    if (_measureMutex == nullptr) {
        ESP_LOGE(LogTag, "Failed to create measure mutex");
        return false;
    }

    if (_measureTaskHandle == nullptr && xTaskCreatePinnedToCore(
            this->vTaskMeasureLoop,
            "Energy Measure",
            MeasureTaskStackSize,
            this,
            MeasureTaskPriority,
            &_measureTaskHandle,
            PRO_CPU_NUM
        ) != pdPASS
    ) {
        ESP_LOGE(LogTag, "Failed to create measure task");
        return false;
    }

    _beginCalled = true;
    ESP_LOGI(LogTag, "Energy monitor started: SDA=%d SCL=%d period=%lu ms", _pinSda, _pinScl, static_cast<unsigned long>(MeasurePeriodMs));
    return true;
}

bool EnergyMonitor::getMeasure(Measure& measure, bool resetWindow) {
    if (!this->readMeasure(_accumulators, measure, resetWindow)) {
        return false;
    }

    ESP_LOGI(
        LogTag,
        "Measure read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh window=%.2f s",
        measure.energy, measure.averageCurrent, measure.averageVoltage,
        measure.accumulatedCharge, measure.elapsedTime
    );
    return true;
}

bool EnergyMonitor::getCycleMeasure(Measure& measure, bool resetCycle) {
    if (!this->readMeasure(_cycleAccumulators, measure, resetCycle)) {
        return false;
    }

    ESP_LOGI(
        LogTag,
        "Cycle measure read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh cycle=%.2f s",
        measure.energy, measure.averageCurrent, measure.averageVoltage,
        measure.accumulatedCharge, measure.elapsedTime
    );
    return true;
}

float EnergyMonitor::getEnergy() {
    Measure measure;
    return this->readMeasure(_accumulators, measure, false) ? measure.energy : NAN;
}

float EnergyMonitor::getAverageCurrent() {
    Measure measure;
    return this->readMeasure(_accumulators, measure, false) ? measure.averageCurrent : NAN;
}

float EnergyMonitor::getAverageVoltage() {
    Measure measure;
    return this->readMeasure(_accumulators, measure, false) ? measure.averageVoltage : NAN;
}

float EnergyMonitor::getAccumulatedCharge() {
    Measure measure;
    return this->readMeasure(_accumulators, measure, false) ? measure.accumulatedCharge : NAN;
}

void EnergyMonitor::resetMeasure() {
    Measure measure;
    this->readMeasure(_accumulators, measure, true);
}

bool EnergyMonitor::readMeasure(MeasureAccumulators& accumulators, Measure& measure, bool resetWindow) {
    if (!_beginCalled) {
        return false;
    }

    xSemaphoreTake(_measureMutex, portMAX_DELAY);
    const MeasureAccumulators window = accumulators;
    if (resetWindow) {
        accumulators = {};
    }
    xSemaphoreGive(_measureMutex);

    measure.energy = window.energy;
    measure.accumulatedCharge = window.charge * 1e3 / SecondsPerHour;
    measure.elapsedTime = window.time;
    measure.averageCurrent = window.time > 0.0 ? window.charge * 1e3 / window.time : NAN;
    measure.averageVoltage = window.time > 0.0 ? window.voltageTime / window.time : NAN;
    return true;
}

void EnergyMonitor::vTaskMeasureLoop(void* pvParameters) {
    auto self = static_cast<EnergyMonitor*>(pvParameters);
    TickType_t lastWake = xTaskGetTickCount();
    int64_t lastTimeUs = esp_timer_get_time();

    while (true) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MeasurePeriodMs));

        xSemaphoreTake(self->_measureMutex, portMAX_DELAY);
        double current = 0.0;
        double voltage = 0.0;
        if (self->readSample(current, voltage)) {
            const int64_t currentTimeUs = esp_timer_get_time();
            const double dt = (currentTimeUs - lastTimeUs) * 1e-6;
            lastTimeUs = currentTimeUs;

            _accumulators.energy += voltage * current * dt;
            _accumulators.charge += current * dt;
            _accumulators.voltageTime += voltage * dt;
            _accumulators.time += dt;

            _cycleAccumulators.energy += voltage * current * dt;
            _cycleAccumulators.charge += current * dt;
            _cycleAccumulators.voltageTime += voltage * dt;
            _cycleAccumulators.time += dt;
            self->onSample();
        }
        xSemaphoreGive(self->_measureMutex);
    }
}

void EnergyMonitor::onSample() {}

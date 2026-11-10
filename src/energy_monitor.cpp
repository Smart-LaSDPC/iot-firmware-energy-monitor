#include "energy_monitor.h"

#include <cmath>
#include <esp_timer.h>
#include <esp_log.h>

RTC_DATA_ATTR EnergyMonitor::MeasureAccumulators EnergyMonitor::_accumulators = {};

EnergyMonitor::EnergyMonitor(int pinSda, int pinScl, uint8_t address, TwoWire& wire)
    : _sensor(address, &wire),
      _wire(wire),
      _pinSda(pinSda),
      _pinScl(pinScl) {}

EnergyMonitor::~EnergyMonitor() {
    if (_measureTaskHandle != nullptr) {
        xSemaphoreTake(_measureMutex, portMAX_DELAY);
        vTaskDelete(_measureTaskHandle);
        xSemaphoreGive(_measureMutex);
    }
    if (_measureMutex != nullptr) {
        vSemaphoreDelete(_measureMutex);
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

    if (!_sensor.begin()) {
        ESP_LOGE(LogTag, "INA226 not found at address 0x%02X", _sensor.getAddress());
        return false;
    }

    const int configureError = _sensor.configure(ShuntResistanceOhm, CurrentLsbMa);
    if (configureError != INA226_ERR_NONE) {
        ESP_LOGE(LogTag, "Failed to configure INA226: error=0x%04X", configureError);
        return false;
    }

    if (!_sensor.setBusVoltageConversionTime(INA226_588_us)) {
        ESP_LOGE(LogTag, "Failed to set INA226 bus voltage conversion time");
        return false;
    }

    if (!_sensor.setShuntVoltageConversionTime(INA226_588_us)) {
        ESP_LOGE(LogTag, "Failed to set INA226 shunt voltage conversion time");
        return false;
    }

    if (!_sensor.setAverage(INA226_4_SAMPLES)) {
        ESP_LOGE(LogTag, "Failed to set INA226 averaging");
        return false;
    }

    if (!_sensor.setModeShuntBusContinuous()) {
        ESP_LOGE(LogTag, "Failed to set INA226 continuous mode");
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
    ESP_LOGI(LogTag, "Energy monitor started: SDA=%d SCL=%d period=%u ms", _pinSda, _pinScl, MeasurePeriodMs);
    return true;
}

bool EnergyMonitor::getMeasure(Measure& measure, bool resetWindow) {
    if (!this->readMeasure(measure, resetWindow)) {
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

float EnergyMonitor::getEnergy() {
    Measure measure;
    return this->readMeasure(measure, false) ? measure.energy : NAN;
}

float EnergyMonitor::getAverageCurrent() {
    Measure measure;
    return this->readMeasure(measure, false) ? measure.averageCurrent : NAN;
}

float EnergyMonitor::getAverageVoltage() {
    Measure measure;
    return this->readMeasure(measure, false) ? measure.averageVoltage : NAN;
}

float EnergyMonitor::getAccumulatedCharge() {
    Measure measure;
    return this->readMeasure(measure, false) ? measure.accumulatedCharge : NAN;
}

void EnergyMonitor::resetMeasure() {
    Measure measure;
    this->readMeasure(measure, true);
}

bool EnergyMonitor::readMeasure(Measure& measure, bool resetWindow) {
    if (!_beginCalled) {
        return false;
    }

    xSemaphoreTake(_measureMutex, portMAX_DELAY);
    const MeasureAccumulators window = _accumulators;
    if (resetWindow) {
        _accumulators = {};
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
        if (self->_sensor.isConversionReady()) {
            const double current = self->_sensor.getCurrent();
            const double voltage = self->_sensor.getBusVoltage();

            const int64_t currentTimeUs = esp_timer_get_time();
            const double dt = (currentTimeUs - lastTimeUs) * 1e-6;
            lastTimeUs = currentTimeUs;

            _accumulators.energy += voltage * current * dt;
            _accumulators.charge += current * dt;
            _accumulators.voltageTime += voltage * dt;
            _accumulators.time += dt;
        }
        xSemaphoreGive(self->_measureMutex);
    }
}

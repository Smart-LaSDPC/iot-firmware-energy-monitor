#include "energy_monitor.h"
#include "basic_energy_monitor.h"
#include "complete_energy_monitor.h"

#include <cmath>
#include <new>
#include <esp_timer.h>
#include <esp_log.h>

static constexpr const char* LogTag = EnergyMonitorSensor::LogTag;
static constexpr double SecondsPerHour = EnergyMonitorSensor::SecondsPerHour;

static EnergyMonitorSensor* createSensor(bool monitoringInSleep, uint8_t address, TwoWire& wire) {
    if (monitoringInSleep) {
        return new (std::nothrow) CompleteEnergyMonitor(address, wire);
    } else {
        return new (std::nothrow) BasicEnergyMonitor(address, wire);
    }
}

RTC_DATA_ATTR EnergyMonitor::MeasureAccumulators EnergyMonitor::_accumulators = {};
RTC_DATA_ATTR EnergyMonitor::MeasureAccumulators EnergyMonitor::_cycleAccumulators = {};

EnergyMonitor::EnergyMonitor(int pinSda, int pinScl, bool monitoringInSleep, uint8_t address, TwoWire& wire)
    : _wire(wire),
      _monitoringInSleep(monitoringInSleep),
      _sensor(createSensor(monitoringInSleep, address, wire)),
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

    if (_sensor == nullptr) {
        ESP_LOGE(LogTag, "Failed to allocate %s sensor", _monitoringInSleep ? "INA228" : "INA226");
        return false;
    }

    if (!_wire.begin(_pinSda, _pinScl, I2cFrequency)) {
        ESP_LOGE(LogTag, "Failed to start I2C bus: SDA=%d SCL=%d", _pinSda, _pinScl);
        return false;
    }

    if (!_sensor->begin()) {
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
    ESP_LOGI(
        LogTag,
        "Energy monitor started: sensor=%s SDA=%d SCL=%d period=%lu ms",
        _monitoringInSleep ? "INA228" : "INA226", _pinSda, _pinScl, static_cast<unsigned long>(MeasurePeriodMs)
    );
    return true;
}

bool EnergyMonitor::getConsumption(EnergyMonitorData& consumption, bool resetWindow) {
    if (!this->readConsumption(_accumulators, consumption, resetWindow)) {
        return false;
    }

    ESP_LOGI(
        LogTag,
        "Consumption read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh window=%.2f s",
        consumption.energy, consumption.averageCurrent, consumption.averageVoltage,
        consumption.accumulatedCharge, consumption.elapsedTime
    );
    return true;
}

bool EnergyMonitor::getCycleConsumption(EnergyMonitorData& consumption, bool resetCycle) {
    if (!this->readConsumption(_cycleAccumulators, consumption, resetCycle)) {
        return false;
    }

    ESP_LOGI(
        LogTag,
        "Cycle consumption read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh cycle=%.2f s",
        consumption.energy, consumption.averageCurrent, consumption.averageVoltage,
        consumption.accumulatedCharge, consumption.elapsedTime
    );
    return true;
}

bool EnergyMonitor::getSleepConsumption(EnergyMonitorData& consumption) {
    if (!_monitoringInSleep) {
        ESP_LOGD(LogTag, "Sleep consumption unavailable: monitoring in sleep disabled (INA226)");
        return false;
    }

    if (!_beginCalled) {
        return false;
    }

    if (!_sensor->getSleepConsumption(consumption)) {
        return false;
    }

    ESP_LOGI(
        LogTag,
        "Sleep consumption read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh window=%.2f s",
        consumption.energy, consumption.averageCurrent, consumption.averageVoltage,
        consumption.accumulatedCharge, consumption.elapsedTime
    );
    return true;
}

bool EnergyMonitor::isMonitoringInSleep() const {
    return _monitoringInSleep;
}

float EnergyMonitor::getEnergy() {
    EnergyMonitorData consumption;
    return this->readConsumption(_accumulators, consumption, false) ? consumption.energy : NAN;
}

float EnergyMonitor::getAverageCurrent() {
    EnergyMonitorData consumption;
    return this->readConsumption(_accumulators, consumption, false) ? consumption.averageCurrent : NAN;
}

float EnergyMonitor::getAverageVoltage() {
    EnergyMonitorData consumption;
    return this->readConsumption(_accumulators, consumption, false) ? consumption.averageVoltage : NAN;
}

float EnergyMonitor::getAccumulatedCharge() {
    EnergyMonitorData consumption;
    return this->readConsumption(_accumulators, consumption, false) ? consumption.accumulatedCharge : NAN;
}

void EnergyMonitor::resetConsumption() {
    EnergyMonitorData consumption;
    this->readConsumption(_accumulators, consumption, true);
}

bool EnergyMonitor::readConsumption(MeasureAccumulators& accumulators, EnergyMonitorData& consumption, bool resetWindow) {
    if (!_beginCalled) {
        return false;
    }

    xSemaphoreTake(_measureMutex, portMAX_DELAY);
    const MeasureAccumulators window = accumulators;
    if (resetWindow) {
        accumulators = {};
    }
    xSemaphoreGive(_measureMutex);

    consumption.energy = window.energy;
    consumption.accumulatedCharge = window.charge * 1e3 / SecondsPerHour;
    consumption.elapsedTime = window.time;
    consumption.averageCurrent = window.time > 0.0 ? window.charge * 1e3 / window.time : NAN;
    consumption.averageVoltage = window.time > 0.0 ? window.voltageTime / window.time : NAN;
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
        if (self->_sensor->readSample(current, voltage)) {
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
            self->_sensor->onSample();
        }
        xSemaphoreGive(self->_measureMutex);
    }
}

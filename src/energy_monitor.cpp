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
RTC_DATA_ATTR EnergyMonitor::MeasureAccumulators EnergyMonitor::_wakeUpAccumulators = {};
RTC_DATA_ATTR EnergyMonitorData EnergyMonitor::_savedWakeUpConsumption = {};
RTC_DATA_ATTR bool EnergyMonitor::_hasSavedWakeUpConsumption = false;

EnergyMonitor::EnergyMonitor(
    int pinSda,
    int pinScl,
    unsigned long monitorResolutionMs,
    bool monitoringInSleep,
    uint8_t address,
    TwoWire& wire
)
    : _wire(wire),
      _monitoringInSleep(monitoringInSleep),
      _monitorResolutionMs(monitorResolutionMs),
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

    if (!_sleepAddedToWindow) {
        this->addSleepToWindow();
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
        _monitoringInSleep ? "INA228" : "INA226", _pinSda, _pinScl, _monitorResolutionMs
    );
    return true;
}

bool EnergyMonitor::getConsumption(EnergyMonitorData& consumption, bool resetWindow) {
    if (!this->readConsumption(_accumulators, consumption, resetWindow)) {
        ESP_LOGE(LogTag, "Consumption read failed: monitor not started");
        return false;
    }

    ESP_LOGI(
        LogTag,
        "Consumption read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh window=%.2f ms",
        consumption.energy, consumption.averageCurrent, consumption.averageVoltage,
        consumption.accumulatedCharge, consumption.elapsedTime
    );
    return true;
}

bool EnergyMonitor::saveWakeUpConsumption() {
    EnergyMonitorData consumption = {};

    if (!this->readConsumption(_wakeUpAccumulators, consumption, true)) {
        ESP_LOGE(LogTag, "Wake up consumption not saved: monitor not started");
        return false;
    }

    if (consumption.elapsedTime == 0) {
        _hasSavedWakeUpConsumption = false;
        ESP_LOGW(LogTag, "Wake up consumption not saved: wake window has no samples");
        return false;
    }

    _savedWakeUpConsumption = consumption;
    _hasSavedWakeUpConsumption = true;

    ESP_LOGI(
        LogTag,
        "Wake up consumption saved: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh wake=%lu ms",
        consumption.energy, consumption.averageCurrent, consumption.averageVoltage,
        consumption.accumulatedCharge, static_cast<unsigned long>(consumption.elapsedTime)
    );
    return true;
}

bool EnergyMonitor::getWakeUpConsumption(EnergyMonitorData& consumption) {
    if (!_hasSavedWakeUpConsumption) {
        ESP_LOGD(LogTag, "Wake up consumption unavailable: none saved since power-on");
        return false;
    }

    consumption = _savedWakeUpConsumption;

    ESP_LOGI(
        LogTag,
        "Wake up consumption read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh wake=%lu ms",
        consumption.energy, consumption.averageCurrent, consumption.averageVoltage,
        consumption.accumulatedCharge, static_cast<unsigned long>(consumption.elapsedTime)
    );
    return true;
}

bool EnergyMonitor::getSleepConsumption(EnergyMonitorData& consumption) {
    if (!_monitoringInSleep) {
        ESP_LOGD(LogTag, "Sleep consumption unavailable: monitoring in sleep disabled (INA226)");
        return false;
    }

    if (!_beginCalled) {
        ESP_LOGE(LogTag, "Sleep consumption unavailable: monitor not started");
        return false;
    }

    if (!_sensor->getSleepConsumption(consumption)) {
        ESP_LOGD(LogTag, "Sleep consumption unavailable: no valid sleep window");
        return false;
    }

    ESP_LOGI(
        LogTag,
        "Sleep consumption read: energy=%.6f J current=%.3f mA voltage=%.3f V charge=%.6f mAh window=%.2f ms",
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

void EnergyMonitor::addSleepToWindow() {
    EnergyMonitorData sleep = {};

    _sleepAddedToWindow = true;

    if (_sensor->getSleepConsumption(sleep)) {
        if (!std::isnan(sleep.averageVoltage)) {
            xSemaphoreTake(_measureMutex, portMAX_DELAY);
            _accumulators.energy += sleep.energy;
            _accumulators.charge += sleep.accumulatedCharge * SecondsPerHour / 1e3;
            const double sleepElapsedTimeS = sleep.elapsedTime * 1e-3;
            _accumulators.voltageTime += sleep.averageVoltage * sleepElapsedTimeS;
            _accumulators.time += sleepElapsedTimeS;
            xSemaphoreGive(_measureMutex);

            ESP_LOGD(
                LogTag, "Sleep added to the consumption window: energy=%.6f J charge=%.6f mAh sleep=%.2f ms",
                sleep.energy, sleep.accumulatedCharge, sleep.elapsedTime
            );
        } else {
            ESP_LOGW(LogTag, "Sleep not added to the consumption window: no charge measured in sleep");
        }
    } else {
        ESP_LOGD(LogTag, "No sleep consumption to add to the consumption window");
    }
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
    if (window.time > static_cast<double>(UINT32_MAX) * 1e-3) {
        consumption.elapsedTime = UINT32_MAX;
    } else {
        consumption.elapsedTime = static_cast<uint32_t>(window.time * 1e3);
    }
    consumption.averageCurrent = window.time > 0.0 ? window.charge * 1e3 / window.time : NAN;
    consumption.averageVoltage = window.time > 0.0 ? window.voltageTime / window.time : NAN;
    return true;
}

void EnergyMonitor::vTaskMeasureLoop(void* pvParameters) {
    auto self = static_cast<EnergyMonitor*>(pvParameters);
    TickType_t lastWake = xTaskGetTickCount();
    int64_t lastTimeUs = esp_timer_get_time();

    while (true) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(self->_monitorResolutionMs));

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

            _wakeUpAccumulators.energy += voltage * current * dt;
            _wakeUpAccumulators.charge += current * dt;
            _wakeUpAccumulators.voltageTime += voltage * dt;
            _wakeUpAccumulators.time += dt;
            self->_sensor->onSample();
        }
        xSemaphoreGive(self->_measureMutex);
    }
}

#include "basic_energy_monitor.h"

#include <esp_log.h>

static constexpr float CurrentLsbMa = 25e-3f;

BasicEnergyMonitor::BasicEnergyMonitor(int pinSda, int pinScl, uint8_t address, TwoWire& wire)
    : EnergyMonitor(pinSda, pinScl, wire),
      _sensor(address, &wire) {}

BasicEnergyMonitor::~BasicEnergyMonitor() {
    this->stop();
}

bool BasicEnergyMonitor::beginSensor() {
    if (!this->_sensor.begin()) {
        ESP_LOGE(LogTag, "INA226 not found at address 0x%02X", this->_sensor.getAddress());
        return false;
    }

    const int configureError = this->_sensor.configure(ShuntResistanceOhm, CurrentLsbMa);
    if (configureError != INA226_ERR_NONE) {
        ESP_LOGE(LogTag, "Failed to configure INA226: error=0x%04X", configureError);
        return false;
    }

    if (!this->_sensor.setBusVoltageConversionTime(INA226_588_us)) {
        ESP_LOGE(LogTag, "Failed to set INA226 bus voltage conversion time");
        return false;
    }

    if (!this->_sensor.setShuntVoltageConversionTime(INA226_588_us)) {
        ESP_LOGE(LogTag, "Failed to set INA226 shunt voltage conversion time");
        return false;
    }

    if (!this->_sensor.setAverage(INA226_4_SAMPLES)) {
        ESP_LOGE(LogTag, "Failed to set INA226 averaging");
        return false;
    }

    if (!this->_sensor.setModeShuntBusContinuous()) {
        ESP_LOGE(LogTag, "Failed to set INA226 continuous mode");
        return false;
    }

    return true;
}

bool BasicEnergyMonitor::readSample(double& currentA, double& voltageV) {
    if (!this->_sensor.isConversionReady()) {
        return false;
    }

    currentA = this->_sensor.getCurrent();
    voltageV = this->_sensor.getBusVoltage();
    return true;
}

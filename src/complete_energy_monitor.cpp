#include "complete_energy_monitor.h"

#include <cmath>
#include <esp_log.h>
#include <esp_system.h>
#if __has_include(<esp_rtc_time.h>)
#include <esp_rtc_time.h>
#else
#include "esp32/rtc.h"
#endif

static constexpr float MaxCurrentA = 1.6384f; // Fundo de escala com ADCRANGE 0: 163,84 mV / 0,1 Ω

RTC_DATA_ATTR CompleteEnergyMonitor::SleepSnapshot CompleteEnergyMonitor::sleepSnapshot = {};

CompleteEnergyMonitor::CompleteEnergyMonitor(uint8_t address, TwoWire& wire)
    : _sensor(address, &wire) {}

bool CompleteEnergyMonitor::begin() {
    if (!this->_sensor.begin()) {
        ESP_LOGE(LogTag, "INA228 not found at address 0x%02X", this->_sensor.getAddress());
        return false;
    }

    if (this->_sensor.setMaxCurrentShunt(MaxCurrentA, ShuntResistanceOhm) != 0) {
        ESP_LOGE(LogTag, "Failed to configure INA228 shunt calibration");
        return false;
    }

    if (this->_sensor.getADCRange()) {
        if (!this->_sensor.setADCRange(false)) {
            ESP_LOGE(LogTag, "Failed to set INA228 ADC range");
            return false;
        }
    }

    if (!this->_sleepConsumptionComputed) {
        this->computeSleepConsumption();
    }

    // O datasheet não garante que RSTACC volte a 0 sozinho, então é limpo em seguida
    if (!this->_sensor.setAccumulation(1) || !this->_sensor.setAccumulation(0)) {
        ESP_LOGE(LogTag, "Failed to clear INA228 energy and charge accumulators");
        return false;
    }

    if (!this->_sensor.setBusVoltageConversionTime(INA228_540_us) ||
        !this->_sensor.setShuntVoltageConversionTime(INA228_540_us) ||
        !this->_sensor.setAverage(INA228_4_SAMPLES) ||
        !this->_sensor.setMode(INA228_MODE_CONT_BUS_SHUNT)) {
        ESP_LOGE(LogTag, "Failed to configure INA228 ADC");
        return false;
    }

    sleepSnapshot.energyJ = this->_sensor.getEnergy();
    sleepSnapshot.chargeC = this->_sensor.getCharge();
    sleepSnapshot.rtcTimeUs = esp_rtc_get_time_us();
    sleepSnapshot.magic = SleepSnapshotMagic;
    return true;
}

void CompleteEnergyMonitor::computeSleepConsumption() {
    this->_sleepConsumptionComputed = true;
    this->_sleepConsumptionValid = false;

    const esp_reset_reason_t resetReason = esp_reset_reason();

    if (resetReason == ESP_RST_DEEPSLEEP) {
        if (sleepSnapshot.magic != SleepSnapshotMagic) {
            ESP_LOGW(LogTag, "INA228 sleep snapshot is not available");
            return;
        }
        
        const uint64_t rtcNowUs = esp_rtc_get_time_us();
        const uint64_t dTimeUs = rtcNowUs > sleepSnapshot.rtcTimeUs ? rtcNowUs - sleepSnapshot.rtcTimeUs : 0;
        
        const double dEnergyJ = this->_sensor.getEnergy() - sleepSnapshot.energyJ;
        const double dChargeC = this->_sensor.getCharge() - sleepSnapshot.chargeC;
        
        if (dEnergyJ >= 0.0 && dChargeC >= 0.0 && dTimeUs > 0) {
            this->_sleepConsumption.elapsedTime = dTimeUs * 1e-6;
            
            this->_sleepConsumption.energy = dEnergyJ;
            this->_sleepConsumption.accumulatedCharge = dChargeC * 1e3 / SecondsPerHour;
            this->_sleepConsumption.averageCurrent = dChargeC * 1e3 / this->_sleepConsumption.elapsedTime;
            this->_sleepConsumption.averageVoltage = dChargeC > 0.0 ? dEnergyJ / dChargeC : NAN;
            
            this->_sleepConsumptionValid = true;
        } else {
            ESP_LOGW(LogTag, "INA228 sleep accumulators are invalid after deep sleep");
        }
    } else {
        ESP_LOGI(LogTag, "No INA228 sleep consumption on reset reason %d", static_cast<int>(resetReason));
    }
}

bool CompleteEnergyMonitor::readSample(double& currentA, double& voltageV) {
    if (this->_sensor.getDiagnoseAlertBit(INA228_DIAG_CONVERT_COMPLETE) == 0) {
        return false;
    }

    currentA = this->_sensor.getCurrent();
    voltageV = this->_sensor.getBusVoltage();
    return true;
}

void CompleteEnergyMonitor::onSample() {
    sleepSnapshot.energyJ = this->_sensor.getEnergy();
    sleepSnapshot.chargeC = this->_sensor.getCharge();
    sleepSnapshot.rtcTimeUs = esp_rtc_get_time_us();
    sleepSnapshot.magic = SleepSnapshotMagic;
}

bool CompleteEnergyMonitor::getSleepConsumption(EnergyMonitorData& consumption) {
    if (!this->_sleepConsumptionValid) {
        return false;
    }

    consumption = this->_sleepConsumption;
    return true;
}

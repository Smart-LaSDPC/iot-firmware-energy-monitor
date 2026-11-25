/*
    Uso com INA228: imprime as medidas do último sono e do ciclo, mede por 3 s e dorme por 10 s.
*/
#include <Arduino.h>
#include "energy_monitor.h"
#include <esp_sleep.h>

static constexpr const char* LogTag = "Example";
static constexpr int PinSda = 21;
static constexpr int PinScl = 22;
static constexpr unsigned long MonitorResolutionMs = 10;
static constexpr bool MonitoringInSleep = true;

static EnergyMonitor Monitor(PinSda, PinScl, MonitorResolutionMs, MonitoringInSleep);

void setup() {
    Serial.begin(115200);

    if (!Monitor.begin()) {
        ESP_LOGE(LogTag, "Failed to start complete energy monitor");
        return;
    }

    EnergyMonitorData sleepConsumption;
    if (Monitor.getSleepConsumption(sleepConsumption)) {
        ESP_LOGI(
            LogTag,
            "Sleep: %.6f J %.3f mA %.3f V %.6f mAh %.2f ms",
            sleepConsumption.energy,
            sleepConsumption.averageCurrent,
            sleepConsumption.averageVoltage,
            sleepConsumption.accumulatedCharge,
            sleepConsumption.elapsedTime
        );
    } else {
        ESP_LOGI(LogTag, "No valid INA228 sleep consumption");
    }

    EnergyMonitorData wakeUpConsumption;
    if (Monitor.getWakeUpConsumption(wakeUpConsumption)) {
        ESP_LOGI(LogTag, "Previous wake: %.6f J %.6f mAh", wakeUpConsumption.energy, wakeUpConsumption.accumulatedCharge);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
}

void loop() {
    Monitor.saveWakeUpConsumption();

    ESP_LOGI(LogTag, "Entering deep sleep");
    esp_deep_sleep(10ULL * 1000000ULL);
}

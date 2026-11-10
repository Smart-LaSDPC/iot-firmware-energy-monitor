/*
    Uso básico: inicia o monitor e, a cada 10 s, lê as medidas da janela e inicia uma nova.
*/
#include <Arduino.h>
#include "energy_monitor.h"

static constexpr const char* LogTag = "Example";
static constexpr int PinSda = 21;
static constexpr int PinScl = 22;

static EnergyMonitor Monitor(PinSda, PinScl);

void setup() {
    Serial.begin(115200);

    if (!Monitor.begin()) {
        ESP_LOGE(LogTag, "Failed to start energy monitor");
    }
}

void loop() {
    EnergyMonitor::Measure measure;

    vTaskDelay(pdMS_TO_TICKS(10000));

    // Sem reset: a janela continua acumulando
    ESP_LOGI(LogTag, "Partial charge: %.6f mAh", Monitor.getAccumulatedCharge());

    // Com reset: lê a janela completa e começa outra
    if (!Monitor.getMeasure(measure)) {
        ESP_LOGW(LogTag, "Energy monitor not started");
    }
}

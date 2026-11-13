# iot-firmware-energy-monitor

Monitor de energia para firmwares ESP32 com o sensor INA226. Uma task interna, de prioridade 1 e fixada no `PRO_CPU`, lê corrente e tensão a cada 10 ms e integra as amostras no tempo. A cada leitura, o monitor entrega as medidas da janela, que vai do último reset até a leitura:

| Medida | Unidade | Cálculo |
| --- | --- | --- |
| `energy` | J | ∫ V·I dt |
| `averageCurrent` | mA | ∫ I dt / T |
| `averageVoltage` | V | ∫ V dt / T |
| `accumulatedCharge` | mAh | ∫ I dt / 3600 |
| `elapsedTime` | s | T |

As médias são ponderadas pelo tempo: uma amostra que demorou mais pesa mais. Sem amostras na janela, as médias valem `NaN`.

Os acumuladores ficam na memória RTC, então a janela sobrevive ao deep sleep (o tempo dormindo não entra na janela). Por isso só uma instância deve existir.

## Uso

```cpp
#include "energy_monitor.h"

static EnergyMonitor Monitor(21, 22);   // SDA, SCL, endereço 0x40 e Wire por padrão

void setup() {
    Serial.begin(115200);
    if (!Monitor.begin()) {
        ESP_LOGE("App", "Failed to start energy monitor");
    }
}

void loop() {
    EnergyMonitor::Measure measure;

    vTaskDelay(pdMS_TO_TICKS(10000));
    Monitor.getMeasure(measure);        // lê a janela e inicia uma nova
}
```

- `getMeasure(measure, resetWindow = true)`: lê todas as medidas; com `resetWindow`, zera os acumuladores.
- `getEnergy()`, `getAverageCurrent()`, `getAverageVoltage()`, `getAccumulatedCharge()`: leem uma medida sem iniciar nova janela. Retornam `NaN` antes de `begin()`.
- `resetMeasure()`: inicia uma nova janela sem ler.

## Configuração do INA226

Definida nas constantes privadas de [include/energy_monitor.h](include/energy_monitor.h), igual à do `EnergySensor` de `iot-firmware-sensors`:

- shunt de 0,1 Ω e LSB de corrente de 25 µA;
- conversão de 588 µs para tensão e corrente, com média de 4 amostras;
- I2C a 400 kHz.

## Dependências

- [robtillaart/INA226](https://github.com/RobTillaart/INA226) `^0.6.6`

Exemplo completo em [examples/basic](examples/basic/basic.cpp). A documentação Doxygen está em <https://smart-lasdpc.github.io/iot-firmware-energy-monitor/>, publicada pelo workflow `Doxygen` a cada push na `main`. Para gerar localmente, rode `doxygen Doxyfile` na raiz da biblioteca; as páginas ficam em `doxygen/html`, que é versionada.

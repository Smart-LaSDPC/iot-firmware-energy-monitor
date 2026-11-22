# iot-firmware-energy-monitor

Monitor de energia para firmwares ESP32 com sensores INA226 ou INA228. Uma task interna, de prioridade 1 e fixada no `PRO_CPU`, lê corrente e tensão a cada 10 ms e integra as amostras no tempo. A cada leitura, o monitor entrega as medidas da janela, que vai do último reset até a leitura:

| Medida | Unidade | Cálculo |
| --- | --- | --- |
| `energy` | J | ∫ V·I dt |
| `averageCurrent` | mA | ∫ I dt / T |
| `averageVoltage` | V | ∫ V dt / T |
| `accumulatedCharge` | mAh | ∫ I dt / 3600 |
| `elapsedTime` | s | T |

As médias são ponderadas pelo tempo: uma amostra que demorou mais pesa mais. Sem amostras na janela, as médias valem `NaN`.

Os acumuladores ficam na memória RTC, então a janela sobrevive ao deep sleep (o tempo dormindo não entra na janela). Por isso só uma instância deve existir.

## Hierarquia

- `BasicEnergyMonitor`: implementação do INA226, com a configuração anterior desta biblioteca.
- `CompleteEnergyMonitor`: implementação do INA228, com acumuladores internos de energia e carga.
- `EnergyMonitor`: classe abstrata que contém a task, o mutex e as janelas independentes do sensor.

Use `BasicEnergyMonitor` para manter o comportamento existente.

## Uso básico

```cpp
#include "basic_energy_monitor.h"

static BasicEnergyMonitor Monitor(21, 22);   // SDA, SCL, endereço 0x40 e Wire por padrão

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
- `getCycleMeasure(measure, resetCycle = true)`: lê a janela de ciclo, independente da janela de `getMeasure()`.
- `getSleepMeasure(measure)`: só em `CompleteEnergyMonitor`; lê o que o INA228 acumulou no último deep sleep (ver abaixo).

Para o INA228, troque o include por `complete_energy_monitor.h` e a classe por `CompleteEnergyMonitor`; o resto da API é igual.

## Configuração do INA226

`BasicEnergyMonitor` usa a configuração de [src/basic_energy_monitor.cpp](src/basic_energy_monitor.cpp), igual à do `EnergySensor` de `iot-firmware-sensors`:

- shunt de 0,1 Ω e LSB de corrente de 25 µA;
- conversão de 588 µs para tensão e corrente, com média de 4 amostras;
- I2C a 400 kHz.

## Configuração do INA228

`CompleteEnergyMonitor` usa shunt de 0,1 Ω, ADCRANGE 0 (±163,84 mV), corrente máxima de 1,6384 A, conversões de 540 µs para barramento e shunt, média de 4 amostras e modo contínuo de barramento + shunt.

`getSleepMeasure(measure)` lê a energia e a carga acumuladas pelo hardware enquanto o ESP32 esteve em deep sleep. Ela só é válida após wake de deep sleep quando o INA228 permaneceu alimentado, os acumuladores não foram reiniciados e o snapshot anterior pode ser confirmado.

A janela de sono vai da última amostra anterior ao sono até `begin()` após o wake. Pode incluir até um período de 10 ms antes do sono e o tempo de inicialização até `begin()`. O INA228 consome aproximadamente 0,64 mA em operação contínua durante o sono. O clock lento RTC pode afetar `elapsedTime` e a corrente média calculada. O primeiro boot após power-on não tem medida de sono.

A duração do sono vem do relógio RTC (`esp_rtc_get_time_us()`), que não é alterado por sincronização de hora. Nenhuma chamada é necessária antes de dormir: a task atualiza o snapshot dos acumuladores a cada amostra.

## Migração da versão 1.x

`EnergyMonitor` agora é abstrata. Troque:

```cpp
EnergyMonitor Monitor(21, 22);
extern EnergyMonitor Monitor;
```

por:

```cpp
BasicEnergyMonitor Monitor(21, 22);
extern BasicEnergyMonitor Monitor;
```

Quando uma referência abstrata for necessária, use `extern EnergyMonitor& Monitor;`.

## Dependências

- [robtillaart/INA226](https://github.com/RobTillaart/INA226) `^0.6.6`
- [robtillaart/INA228](https://github.com/RobTillaart/INA228) `^0.4.1`

Exemplos: [examples/basic](examples/basic/basic.cpp) e [examples/complete](examples/complete/complete.cpp). A documentação Doxygen está em <https://smart-lasdpc.github.io/iot-firmware-energy-monitor/>, publicada pelo workflow `Doxygen` a cada push na `main`. Para gerar localmente, rode `doxygen Doxyfile` na raiz da biblioteca; as páginas ficam em `doxygen/html`, que é versionada.

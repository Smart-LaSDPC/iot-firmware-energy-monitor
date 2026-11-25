#pragma once

/**
 * @file energy_monitor.h
 * @brief Infraestrutura independente do sensor para integração de energia.
 */
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <memory>

class EnergyMonitorSensor;

/** @brief Consumo de uma janela. */
struct EnergyMonitorData {
    float energy;               ///< Energia consumida, em J.
    float averageCurrent;       ///< Corrente média, em mA. NaN se a janela não tem amostras.
    float averageVoltage;       ///< Tensão média do barramento, em V. NaN se a janela não tem amostras.
    float accumulatedCharge;    ///< Corrente acumulada, em mAh.
    uint32_t elapsedTime;       ///< Duração da janela, em ms.
};

/**
 * @brief Monitor que amostra periodicamente um sensor e acumula as medidas
 * da janela atual, que vai do último reset até a leitura.
 *
 * O sensor é escolhido no construtor: com monitoringInSleep false, usa o INA226
 * (BasicEnergyMonitor), sem medida de sono; com true, usa o INA228
 * (CompleteEnergyMonitor), que também mede o último deep sleep.
 *
 * Os acumuladores ficam na memória RTC, então a janela sobrevive ao deep sleep. Por
 * isso só uma instância deve existir: todas compartilham os mesmos acumuladores.
 * Uma segunda janela, a do wake, é integrada pela mesma task, guardada por saveWakeUpConsumption()
 * antes do deep sleep e lida no wake seguinte por getWakeUpConsumption().
 */
class EnergyMonitor {
  public:
    /**
     * @brief Define os pinos, o barramento I2C e o sensor usados pelo monitor.
     * @param pinSda Pino SDA do barramento I2C.
     * @param pinScl Pino SCL do barramento I2C.
     * @param monitorResolutionMs Intervalo de amostragem, em milissegundos.
     * @param monitoringInSleep true para usar o INA228 e medir o deep sleep; false para usar o INA226.
     * @param address Endereço I2C do sensor.
     * @param wire Barramento I2C usado pelo sensor.
     */
    EnergyMonitor(
        int pinSda,
        int pinScl,
        unsigned long monitorResolutionMs = 10,
        bool monitoringInSleep = false,
        uint8_t address = 0x40,
        TwoWire& wire = Wire
    );

    /** @brief Encerra a task de medição, libera o mutex e o sensor. */
    ~EnergyMonitor();

    EnergyMonitor(const EnergyMonitor&) = delete;
    EnergyMonitor& operator=(const EnergyMonitor&) = delete;

    /**
     * @brief Inicia o barramento I2C, configura o sensor e cria o mutex e a task de medição.
     *
     * Pode ser chamado de novo após uma falha: o que já foi criado é reaproveitado.
     *
     * @return true se a inicialização foi concluída; false se o barramento ou o sensor
     * não responderam, ou se faltou memória para o sensor, o mutex ou a task.
     */
    bool begin();

    /**
     * @brief Lê as medidas da janela atual.
     *
     * Com monitoringInSleep (INA228), a janela também inclui o consumo dos deep sleeps desde o
     * último reset dela: begin() soma a medida do último sono uma vez por boot. A janela do wake
     * (saveWakeUpConsumption()) não inclui o sono.
     *
     * @param consumption Recebe as medidas da janela.
     * @param resetWindow Se true, zera os acumuladores e inicia uma nova janela.
     * @return true se as medidas foram lidas; false se begin() não foi concluído.
     */
    bool getConsumption(EnergyMonitorData& consumption, bool resetWindow = true);

    /**
     * @brief Lê e zera a janela do wake, independente da janela de getConsumption(), e guarda o
     * resultado na memória RTC. Chamar antes do deep sleep: no wake seguinte,
     * getWakeUpConsumption() devolve esse valor.
     *
     * @return true se o consumo foi guardado; false se begin() não foi concluído ou se a janela
     * não tem amostras (nada é guardado e o valor anterior é descartado).
     */
    bool saveWakeUpConsumption();

    /**
     * @brief Lê o consumo do wake anterior, guardado por saveWakeUpConsumption() antes do deep sleep.
     *
     * @param consumption Recebe as medidas do wake anterior.
     * @return true se há um consumo guardado; false após power-on/reset, até o primeiro
     * saveWakeUpConsumption() com amostras.
     */
    bool getWakeUpConsumption(EnergyMonitorData& consumption);

    /**
     * @brief Lê o que o INA228 acumulou no último deep sleep.
     *
     * @param consumption Recebe as medidas da janela de sono.
     * @return true somente com monitoringInSleep, após begin() e em um wake válido de deep sleep;
     * false com monitoringInSleep false (INA226), que não mede o sono.
     */
    bool getSleepConsumption(EnergyMonitorData& consumption);

    /** @brief true se o monitor usa o INA228 e mede o deep sleep. */
    bool isMonitoringInSleep() const;

    /** @brief Energia consumida na janela atual, em J, sem iniciar uma nova janela. */
    float getEnergy();

    /** @brief Corrente média da janela atual, em mA, sem iniciar uma nova janela. */
    float getAverageCurrent();

    /** @brief Tensão média da janela atual, em V, sem iniciar uma nova janela. */
    float getAverageVoltage();

    /** @brief Corrente acumulada na janela atual, em mAh, sem iniciar uma nova janela. */
    float getAccumulatedCharge();

    /** @brief Zera os acumuladores e inicia uma nova janela. */
    void resetConsumption();

    /**
     * @brief Task que, a cada período, lê corrente e tensão do sensor quando há
     * uma conversão pronta e integra as amostras no tempo decorrido desde a anterior.
     *
     * @param pvParameters Instância de EnergyMonitor que criou a task.
     */
    static void vTaskMeasureLoop(void* pvParameters);

  private:
    void stop();

    static constexpr uint32_t I2cFrequency = 400000;
    static constexpr uint32_t MeasureTaskStackSize = configMINIMAL_STACK_SIZE + 1024 * 2;
    static constexpr UBaseType_t MeasureTaskPriority = 1;

    TwoWire& _wire;
    const bool _monitoringInSleep;
    unsigned long _monitorResolutionMs;
    std::unique_ptr<EnergyMonitorSensor> _sensor;
    bool _beginCalled = false;

    /** @brief Integrais da janela atual. */
    struct MeasureAccumulators {
        double energy;          ///< ∫ V·I dt, em J.
        double charge;          ///< ∫ I dt, em A·s.
        double voltageTime;     ///< ∫ V dt, em V·s.
        double time;            ///< ∫ dt, em s.
    };

    static MeasureAccumulators _accumulators;          ///< Janela atual; fica na memória RTC para sobreviver ao deep sleep.
    static MeasureAccumulators _wakeUpAccumulators;    ///< Janela do wake, também na memória RTC.
    static EnergyMonitorData _savedWakeUpConsumption;  ///< Wake guardado por saveWakeUpConsumption() (RTC).
    static bool _hasSavedWakeUpConsumption;            ///< _savedWakeUpConsumption tem valor (RTC).

    bool readConsumption(MeasureAccumulators& accumulators, EnergyMonitorData& consumption, bool resetWindow);

    /**
     * @brief Soma em _accumulators a medida do último deep sleep do sensor, quando houver uma válida.
     * Não mexe em _wakeUpAccumulators. Chamado por begin() uma única vez por boot.
     */
    void addSleepToWindow();

    /** @brief true depois que addSleepToWindow() rodou neste boot, para o sono não ser somado de novo. */
    bool _sleepAddedToWindow = false;

    int _pinSda = 0;
    int _pinScl = 0;

    TaskHandle_t _measureTaskHandle = nullptr;
    SemaphoreHandle_t _measureMutex = nullptr;
};

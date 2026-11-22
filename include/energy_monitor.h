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

/**
 * @brief Classe abstrata que amostra periodicamente um sensor e acumula as medidas
 * da janela atual, que vai do último reset até a leitura.
 *
 * Os acumuladores ficam na memória RTC, então a janela sobrevive ao deep sleep. Por
 * isso só uma instância deve existir: todas compartilham os mesmos acumuladores.
 * Uma segunda janela, a de ciclo, é integrada pela mesma task e lida por getCycleMeasure().
 */
class EnergyMonitor {
  public:
    /** @brief Medidas de uma janela. */
    struct Measure {
        float energy;               ///< Energia consumida, em J.
        float averageCurrent;       ///< Corrente média, em mA. NaN se a janela não tem amostras.
        float averageVoltage;       ///< Tensão média do barramento, em V. NaN se a janela não tem amostras.
        float accumulatedCharge;    ///< Corrente acumulada, em mAh.
        float elapsedTime;          ///< Duração da janela, em s.
    };

    /** @brief Encerra a task de medição e libera o mutex. */
    virtual ~EnergyMonitor();

    /**
     * @brief Inicia o barramento I2C, configura o sensor e cria o mutex e a task de medição.
     *
     * Pode ser chamado de novo após uma falha: o que já foi criado é reaproveitado.
     *
     * @return true se a inicialização foi concluída; false se o barramento ou o sensor
     * não responderam, ou se faltou memória para o mutex ou para a task.
     */
    bool begin();

    /**
     * @brief Lê as medidas da janela atual.
     *
     * @param measure Recebe as medidas da janela.
     * @param resetWindow Se true, zera os acumuladores e inicia uma nova janela.
     * @return true se as medidas foram lidas; false se begin() não foi concluído.
     */
    bool getMeasure(Measure& measure, bool resetWindow = true);

    /**
     * @brief Lê as medidas da janela de ciclo, independente da janela de getMeasure().
     * O ciclo vai do último reset até a leitura, e normalmente é lido uma vez por boot.
     *
     * @param measure Recebe as medidas do ciclo.
     * @param resetCycle Se true, zera os acumuladores do ciclo e inicia um novo.
     * @return true se as medidas foram lidas; false se begin() não foi concluído.
     */
    bool getCycleMeasure(Measure& measure, bool resetCycle = true);

    /** @brief Energia consumida na janela atual, em J, sem iniciar uma nova janela. */
    float getEnergy();

    /** @brief Corrente média da janela atual, em mA, sem iniciar uma nova janela. */
    float getAverageCurrent();

    /** @brief Tensão média da janela atual, em V, sem iniciar uma nova janela. */
    float getAverageVoltage();

    /** @brief Corrente acumulada na janela atual, em mAh, sem iniciar uma nova janela. */
    float getAccumulatedCharge();

    /** @brief Zera os acumuladores e inicia uma nova janela. */
    void resetMeasure();

    /**
     * @brief Task que, a cada período, lê corrente e tensão do sensor quando há
     * uma conversão pronta e integra as amostras no tempo decorrido desde a anterior.
     *
     * @param pvParameters Instância de EnergyMonitor que criou a task.
     */
    static void vTaskMeasureLoop(void* pvParameters);

  protected:
    /**
     * @brief Define os pinos e o barramento I2C usados pelo monitor.
     * @param pinSda Pino SDA do barramento I2C.
     * @param pinScl Pino SCL do barramento I2C.
     * @param wire Barramento I2C usado pelo sensor.
     */
    EnergyMonitor(int pinSda, int pinScl, TwoWire& wire);

    /** @brief Configura o sensor após o barramento I2C já estar iniciado. */
    virtual bool beginSensor() = 0;

    /**
     * @brief Lê uma amostra nova do sensor.
     * @param currentA Corrente em A.
     * @param voltageV Tensão de barramento em V.
     * @return true se havia conversão nova e a amostra foi lida; false para não integrar nada.
     */
    virtual bool readSample(double& currentA, double& voltageV) = 0;

    /** @brief Chamado após integrar uma amostra, sob o mutex da task. */
    virtual void onSample();

    /** @brief Para a task e libera recursos. Seguro para chamadas repetidas. */
    void stop();

    static constexpr const char* LogTag = "Energy";        ///< Tag de log do monitor.
    static constexpr float ShuntResistanceOhm = 0.1f;      ///< Resistência do shunt, em Ω.
    static constexpr double SecondsPerHour = 3600.0;       ///< Conversão de A·s para A·h.

    bool _beginCalled = false;  ///< true após begin() concluído.

  private:
    static constexpr uint32_t I2cFrequency = 400000;
    static constexpr uint32_t MeasurePeriodMs = 10;
    static constexpr uint32_t MeasureTaskStackSize = configMINIMAL_STACK_SIZE + 1024 * 2;
    static constexpr UBaseType_t MeasureTaskPriority = 1;

    TwoWire& _wire;

    /** @brief Integrais da janela atual. */
    struct MeasureAccumulators {
        double energy;          ///< ∫ V·I dt, em J.
        double charge;          ///< ∫ I dt, em A·s.
        double voltageTime;     ///< ∫ V dt, em V·s.
        double time;            ///< ∫ dt, em s.
    };

    static MeasureAccumulators _accumulators;          ///< Janela atual; fica na memória RTC para sobreviver ao deep sleep.
    static MeasureAccumulators _cycleAccumulators;     ///< Janela do ciclo, também na memória RTC.

    bool readMeasure(MeasureAccumulators& accumulators, Measure& measure, bool resetWindow);

    int _pinSda = 0;
    int _pinScl = 0;

    TaskHandle_t _measureTaskHandle = nullptr;
    SemaphoreHandle_t _measureMutex = nullptr;
};

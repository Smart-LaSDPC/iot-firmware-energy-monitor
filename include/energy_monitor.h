#pragma once

/**
 * @file energy_monitor.h
 * @brief Monitor de energia com INA226: uma task interna amostra tensão e corrente
 * e integra energia, carga e as médias da janela de medição.
 */
#include <Arduino.h>
#include <Wire.h>
#include <INA226.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/**
 * @brief Lê o INA226 periodicamente em uma task de prioridade 1 e acumula as medidas
 * da janela atual, que vai do último reset até a leitura.
 *
 * Os acumuladores ficam na memória RTC, então a janela sobrevive ao deep sleep. Por
 * isso só uma instância deve existir: todas compartilham os mesmos acumuladores.
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

    /**
     * @brief Define os pinos e o endereço do INA226. O barramento, o sensor e a task
     * só são iniciados em begin().
     *
     * @param pinSda Pino SDA do barramento I2C.
     * @param pinScl Pino SCL do barramento I2C.
     * @param address Endereço I2C do INA226.
     * @param wire Barramento I2C usado pelo sensor.
     */
    EnergyMonitor(int pinSda, int pinScl, uint8_t address = 0x40, TwoWire& wire = Wire);

    /** @brief Encerra a task de medição e libera o mutex. */
    ~EnergyMonitor();

    /**
     * @brief Inicia o barramento I2C, configura o INA226 e cria o mutex e a task de medição.
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
     * @brief Task que, a cada período, lê corrente e tensão do INA226 quando há
     * conversão pronta e integra as amostras no tempo decorrido desde a anterior.
     *
     * @param pvParameters Instância de EnergyMonitor que criou a task.
     */
    static void vTaskMeasureLoop(void* pvParameters);

  private:
    static constexpr const char* LogTag = "EnergyMonitor";

    static constexpr float ShuntResistanceOhm = 0.1f;
    static constexpr float CurrentLsbMa = 25e-3f;
    static constexpr uint32_t I2cFrequency = 400000;
    static constexpr uint32_t MeasurePeriodMs = 10;
    static constexpr uint32_t MeasureTaskStackSize = configMINIMAL_STACK_SIZE + 1024 * 2;
    static constexpr UBaseType_t MeasureTaskPriority = 1;
    static constexpr double SecondsPerHour = 3600.0;

    /** @brief Integrais da janela atual. */
    struct MeasureAccumulators {
        double energy;          ///< ∫ V·I dt, em J.
        double charge;          ///< ∫ I dt, em A·s.
        double voltageTime;     ///< ∫ V dt, em V·s.
        double time;            ///< ∫ dt, em s.
    };

    static MeasureAccumulators _accumulators; // Permanece na memória RTC para sobreviver ao deep sleep

    bool readMeasure(Measure& measure, bool resetWindow);

    INA226 _sensor;
    TwoWire& _wire;
    int _pinSda = 0;
    int _pinScl = 0;

    bool _beginCalled = false;
    TaskHandle_t _measureTaskHandle = nullptr;
    SemaphoreHandle_t _measureMutex = nullptr;
};

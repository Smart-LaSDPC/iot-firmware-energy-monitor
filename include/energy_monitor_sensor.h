#pragma once

/**
 * @file energy_monitor_sensor.h
 * @brief Interface do sensor usado internamente por EnergyMonitor.
 */
#include "energy_monitor.h"

/**
 * @brief Sensor de corrente e tensão amostrado pela task de EnergyMonitor.
 *
 * Não é usado diretamente pelo firmware: EnergyMonitor cria a implementação
 * conforme o argumento monitoringInSleep do construtor.
 */
class EnergyMonitorSensor {
  public:
    virtual ~EnergyMonitorSensor() = default;

    /** @brief Configura o sensor após o barramento I2C já estar iniciado. */
    virtual bool begin() = 0;

    /**
     * @brief Lê uma amostra nova do sensor.
     * @param currentA Corrente em A.
     * @param voltageV Tensão de barramento em V.
     * @return true se havia conversão nova e a amostra foi lida; false para não integrar nada.
     */
    virtual bool readSample(double& currentA, double& voltageV) = 0;

    /** @brief Chamado após integrar uma amostra, sob o mutex da task. */
    virtual void onSample() {}

    /**
     * @brief Lê a medida do último deep sleep.
     * @param consumption Recebe as medidas da janela de sono.
     * @return false quando o sensor não mede o sono.
     */
    virtual bool getSleepConsumption(EnergyMonitorData& consumption) {
        (void)consumption;
        return false;
    }

    static constexpr const char* LogTag = "Energy";        ///< Tag de log do monitor.
    static constexpr float ShuntResistanceOhm = 0.1f;      ///< Resistência do shunt, em Ω.
    static constexpr double SecondsPerHour = 3600.0;       ///< Conversão de A·s para A·h.
};

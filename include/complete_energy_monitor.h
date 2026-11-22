#pragma once

/**
 * @file complete_energy_monitor.h
 * @brief Sensor INA228 do monitor de energia, que também mede o último deep sleep pelos acumuladores do sensor.
 */
#include "energy_monitor_sensor.h"
#include <INA228.h>

/**
 * @brief Implementação INA228, criada por EnergyMonitor quando monitoringInSleep é true.
 *
 * A medida de sono usa os acumuladores de hardware do INA228 e um snapshot persistente
 * na memória RTC. A janela inclui até um período de 10 ms antes do sono e o boot até begin().
 * O INA228 deve permanecer ligado e em modo contínuo durante o sono.
 */
class CompleteEnergyMonitor : public EnergyMonitorSensor {
  public:
    /**
     * @brief Constrói o sensor INA228.
     * @param address Endereço I2C.
     * @param wire Barramento I2C.
     */
    CompleteEnergyMonitor(uint8_t address, TwoWire& wire);

    bool begin() override;
    bool readSample(double& currentA, double& voltageV) override;
    void onSample() override;

    /**
     * @brief Retorna a medida acumulada pelo INA228 enquanto o ESP32 dormiu.
     *
     * Retorna false no primeiro boot sem snapshot válido ou após reinício dos acumuladores.
     * @param consumption Recebe as medidas da janela de sono.
     * @return true somente em um wake válido de deep sleep.
     */
    bool getSleepConsumption(EnergyMonitorData& consumption) override;

  private:
    /** @brief Leitura dos acumuladores do INA228 e instante RTC em que foi feita. */
    struct SleepSnapshot {
        double energyJ;         ///< ENERGY do INA228, em J.
        double chargeC;         ///< CHARGE do INA228, em C.
        uint64_t rtcTimeUs;     ///< Tempo RTC da leitura, em µs.
        uint32_t magic;         ///< SleepSnapshotMagic quando o snapshot é válido.
    };

    static constexpr uint32_t SleepSnapshotMagic = 0x494E4132;

    /** @brief Última leitura dos acumuladores antes do sono (RTC), base da medida de sono no wake. */
    static SleepSnapshot sleepSnapshot;

    void computeSleepConsumption();

    INA228 _sensor;
    bool _sleepConsumptionComputed = false;
    bool _sleepConsumptionValid = false;
    EnergyMonitorData _sleepConsumption = {};
};

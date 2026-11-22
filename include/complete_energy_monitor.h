#pragma once

/**
 * @file complete_energy_monitor.h
 * @brief Monitor de energia com INA228, que também mede o último deep sleep pelos acumuladores do sensor.
 */
#include "energy_monitor.h"
#include <INA228.h>

/**
 * @brief Implementação INA228 do monitor de energia.
 *
 * A medida de sono usa os acumuladores de hardware do INA228 e um snapshot persistente
 * na memória RTC. A janela inclui até um período de 10 ms antes do sono e o boot até begin().
 * O INA228 deve permanecer ligado e em modo contínuo durante o sono.
 */
class CompleteEnergyMonitor : public EnergyMonitor {
  public:
    /**
     * @brief Constrói um monitor INA228.
     * @param pinSda Pino SDA.
     * @param pinScl Pino SCL.
     * @param address Endereço I2C.
     * @param wire Barramento I2C.
     */
    CompleteEnergyMonitor(int pinSda, int pinScl, uint8_t address = 0x40, TwoWire& wire = Wire);
    ~CompleteEnergyMonitor() override;

    /**
     * @brief Retorna a medida acumulada pelo INA228 enquanto o ESP32 dormiu.
     *
     * Retorna false no primeiro boot sem snapshot válido ou após reinício dos acumuladores.
     * @param measure Recebe as medidas da janela de sono.
     * @return true somente em um wake válido de deep sleep.
     */
    bool getSleepMeasure(Measure& measure);

  protected:
    bool beginSensor() override;
    bool readSample(double& currentA, double& voltageV) override;
    void onSample() override;

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

    INA228 _sensor;
    bool _sleepMeasureComputed = false;
    bool _sleepMeasureValid = false;
    Measure _sleepMeasure = {};
};

#pragma once

/**
 * @file basic_energy_monitor.h
 * @brief Monitor de energia com INA226.
 */
#include "energy_monitor.h"
#include <INA226.h>

/**
 * @brief Implementação INA226 do monitor de energia.
 *
 * Configura shunt de 0,1 Ω, LSB de 25 µA, conversões de 588 µs,
 * média de 4 amostras e modo contínuo corrente + tensão.
 */
class BasicEnergyMonitor : public EnergyMonitor {
  public:
    /**
     * @brief Constrói um monitor INA226.
     * @param pinSda Pino SDA.
     * @param pinScl Pino SCL.
     * @param address Endereço I2C.
     * @param wire Barramento I2C.
     */
    BasicEnergyMonitor(int pinSda, int pinScl, uint8_t address = 0x40, TwoWire& wire = Wire);
    ~BasicEnergyMonitor() override;

  protected:
    bool beginSensor() override;
    bool readSample(double& currentA, double& voltageV) override;

  private:
    INA226 _sensor;
};

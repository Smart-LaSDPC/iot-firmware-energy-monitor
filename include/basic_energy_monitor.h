#pragma once

/**
 * @file basic_energy_monitor.h
 * @brief Sensor INA226 do monitor de energia, sem medida de sono.
 */
#include "energy_monitor_sensor.h"
#include <INA226.h>

/**
 * @brief Implementação INA226, criada por EnergyMonitor quando monitoringInSleep é false.
 *
 * Configura shunt de 0,1 Ω, LSB de 25 µA, conversões de 588 µs,
 * média de 4 amostras e modo contínuo corrente + tensão.
 */
class BasicEnergyMonitor : public EnergyMonitorSensor {
  public:
    /**
     * @brief Constrói o sensor INA226.
     * @param address Endereço I2C.
     * @param wire Barramento I2C.
     */
    BasicEnergyMonitor(uint8_t address, TwoWire& wire);

    bool begin() override;
    bool readSample(double& currentA, double& voltageV) override;

  private:
    INA226 _sensor;
};

/*
 * LaserSensor.hpp
 *
 *  Created on: Apr 29, 2024
 *      Author: andre
 */

#ifndef SRC_LASERSENSOR_H_
#define SRC_LASERSENSOR_H_

#include "VL53L4CD_api.h"
#include "stdio.h"
#include "Util/logging.h"

class LaserSensor {
public:
  LaserSensor(GPIO_TypeDef *port, uint16_t pin, Dev_t address, int16_t offset)
      : pin(pin), port(port), address(address), offset(offset) {}
  LaserSensor() {};

  // destructor
  ~LaserSensor() { VL53L4CD_StopRanging(address); }

  void enableSensor() {
    // set the pin to high to enable the sensor
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  }
  void disableSensor() {
    // set the pin to high to disable the sensor
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  }

  int setup() {
    uint16_t sensor_id;
    uint8_t status;
    LOG_INFO("laser", "trying to setup laser with pin: %d\n", pin);

    HAL_Delay(10);

    HAL_GPIO_WritePin(XSHUT_SENSOR_OBSTACLE_GPIO_Port, XSHUT_SENSOR_OBSTACLE_Pin, GPIO_PIN_SET);
    // set I2C address (other unset addresses XSHUT have to be pull to low before)
    // status = VL53L4CD_SetI2CAddress(0x52, address); // 0x52 is the default address
    // if (status) {
    //   LOG_ERROR("laser", "VL53L4CD_SetI2CAddress failed with status %u\n", status);
    //   return status;
    // }
    address = 0x52;

    /* (Optional) Check if there is a VL53L4CD sensor connected */
    LOG_INFO("laser", "Checking for laser sensor at address %x\n", address);
    status = VL53L4CD_GetSensorId(address, &sensor_id);

    if (status || (sensor_id != 0xEBAA)) {
      LOG_ERROR("laser", "VL53L4CD not detected at requested address\n");
      return status;
    }
    LOG_INFO("laser", "VL53L4CD detected at address %x\n", address);

    /* (Mandatory) Init VL53L4CD sensor */
    LOG_INFO("laser", "Initializing laser sensor\n");
    status = VL53L4CD_SensorInit(address);
    if (status) {
      LOG_ERROR("laser", "VL53L4CD ULD Loading failed\n");
      return status;
    }

    // set the offset
    status = VL53L4CD_SetOffset(address, offset);
    if (status) {
      LOG_ERROR("laser", "VL53L4CD_SetOffset failed with status %u\n", status);
      return status;
    }

    status = VL53L4CD_StartRanging(address);
    if (status) {
      LOG_ERROR("laser", "VL53L4CD_StartRanging failed with status %u\n", status);
      return status;
    }

    LOG_INFO("laser", "VL53L4CD ULD ready at address %x ready\n", address);
    return 0;
  }

  int get_dist_mm() {
    int status = update_distance();
    if (status) {
      LOG_WARN("laser", "VL53L4CD_GetResult failed with status %u\n", status);
      return -1;
    }
    return results.distance_mm;
  }

  void clear_interrupt() { VL53L4CD_ClearInterrupt(address); }

  static void scan() {
    /*I2C Bus Scanning*/
    uint16_t sensor_id;
    uint8_t status;

    for (int i = 1; i < 128; i++) {
      status = VL53L4CD_GetSensorId(i, &sensor_id);
      if (!status && (sensor_id == 0xEBAA)) {
        LOG_INFO("laser", "VL53L4CD detected at address %x\n", i);
      }
      HAL_Delay(5);
    }
    LOG_INFO("laser", "end of scan\n\n");
  }

private:
  uint16_t pin;
  GPIO_TypeDef *port;
  Dev_t address;
  VL53L4CD_ResultsData_t results = {};
  int16_t offset;

  int update_distance() {
    // We don't want to read data at too high frequency, so we store previous
    // time and check against HAL_GetTick(). (5ms min)
    static uint32_t last_read_time = 0;
    if (HAL_GetTick() - last_read_time < 5) {
      return 0;
    }

    /* Use polling function to know when a new measurement is ready.
     * Another way can be to wait for HW interrupt raised on PIN 7
     * (GPIO 1) when a new measurement is ready */

    uint8_t isReady;
    uint8_t status = VL53L4CD_CheckForDataReady(address, &isReady);

    if (isReady) {
      /* (Mandatory) Clear HW interrupt to restart measurements */
      VL53L4CD_ClearInterrupt(address);

      /* Read measured distance. RangeStatus = 0 means valid data */
      VL53L4CD_GetResult(address, &results);
    }

    return status;
  }
};

#endif /* SRC_LASERSENSOR_H_ */

///*******************************************************************************
//*
//* Copyright (c) 2020 STMicroelectronics - All Rights Reserved
//*
//* License terms: STMicroelectronics Proprietary in accordance with licensing
//* terms at www.st.com/sla0081
//*
//* STMicroelectronics confidential
//* Reproduction and Communication of this document is strictly prohibited unless
//* specifically authorized in writing by STMicroelectronics.
//*
//*******************************************************************************/
//
//#ifndef _PLATFORM_H_
//#define _PLATFORM_H_
//#pragma once
//
//#include <stdint.h>
//#include <string.h>
//#include "stm32g4xx.h"
//
///**
// * @brief Device instance.
// */
//
//typedef uint16_t Dev_t;
//
///**
// * @brief Error instance.
// */
//typedef uint8_t VL53L4CD_Error;
//
///**
// * @brief If the macro below is defined, the device will be programmed to run
// * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
// */
//
////#define VL53L4CD_I2C_FAST_MODE_PLUS
//
///**
// * @brief Read 32 bits through I2C.
// */
//
//uint8_t VL53L4CD_RdDWord(uint16_t dev, uint16_t registerAddr, uint32_t *value);
//
///**
// * @brief Read 16 bits through I2C.
// */
//
//uint8_t VL53L4CD_RdWord(uint16_t dev, uint16_t registerAddr, uint16_t *value);
//
///**
// * @brief Read 8 bits through I2C.
// */
//
//uint8_t VL53L4CD_RdByte(uint16_t dev, uint16_t registerAddr, uint8_t *value);
//
///**
// * @brief Write 8 bits through I2C.
// */
//
//uint8_t VL53L4CD_WrByte(uint16_t dev, uint16_t registerAddr, uint8_t value);
//
///**
// * @brief Write 16 bits through I2C.
// */
//
//uint8_t VL53L4CD_WrWord(uint16_t dev, uint16_t RegisterAdress, uint16_t value);
//
///**
// * @brief Write 32 bits through I2C.
// */
//
//uint8_t VL53L4CD_WrDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t value);
//
///**
// * @brief Wait during N milliseconds.
// */
//
//uint8_t WaitMs(Dev_t dev, uint32_t time_ms);
//
//
//#endif	// _PLATFORM_H_

/*
 * HTS221.h
 *
 *  Created on: Oct 14, 2017
 *      Author: pt
 */
#ifndef HTS221_H_
#define HTS221_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"


#define HTS221_DevAddr		 0xBE

#define HTS221_SLAVE_ADDRESS 0x5F 		///< I2C Slave Address
#define HTS221_DEVICE_ID 0xBC 			///< Who am i device identifier
#define HTS221_DEFAULTSETTING			(uint8_t)(HTS221_AVGH_32|HTS221_AVGT_16)
#define HTS221_DEFAULTREG1				(uint8_t)(HTS221_PD|HTS221_BDU|HTS221_ODR_125HZ)
#define HTS221_DEFAULTREG2				(uint8_t)(0)

//
/// @name AV_CONF:AVGH
/// Averaged humidity samples configuration
/// @{
#define HTS221_AVGH_4   0b000
#define HTS221_AVGH_8   0b001
#define HTS221_AVGH_16  0b010
#define HTS221_AVGH_32  0b011 			// default
#define HTS221_AVGH_64  0b100
#define HTS221_AVGH_128 0b101
#define HTS221_AVGH_256 0b110
#define HTS221_AVGH_512 0b111
/// @}

/// @name AV_CONF:AVGT
// Averaged temperature samples configuration
/// @{
#define HTS221_AVGT_2   0b000000
#define HTS221_AVGT_4   0b001000
#define HTS221_AVGT_8   0b010000
#define HTS221_AVGT_16  0b011000 		// default
#define HTS221_AVGT_32  0b100000
#define HTS221_AVGT_64  0b101000
#define HTS221_AVGT_128 0b110000
#define HTS221_AVGT_256 0b111000
/// @}

/// @name CTRL_REG1
/// @{
#define HTS221_PD  0b10000000 			//< Power Down control
#define HTS221_BDU 0b100 				//< Block Data Update control
#define HTS221_ODR_ONE   0b00 			//< Output Data Rate : One Shot
#define HTS221_ODR_1HZ   0b01 			//< Output Data Rate : 1Hz
#define HTS221_ODR_7HZ   0b10 			//< Output Data Rate : 7Hz
#define HTS221_ODR_125HZ 0b11 			//< Output Data Rate : 12.5Hz
/// @}

/// @name CTRL_REG2
/// @{
#define HTS221_BOOT  0b10000000 		//< Reboot memory content
#define HTS221_HEATER 0b10 				//< Heater
#define HTS221_ONE_SHOT 0b1 			//< One shot enable
/// @}

/// @name CTRL_REG3
/// @{
#define HTS221_CTRL_REG3_DEFAULT 0x00 	//< DRDY pin is no connect in FaBo Brick
/// @}

/// @name STATUS_REG
/// @{
#define HTS221_H_DA 0x2 				//< Humidity Data Available
#define HTS221_T_DA 0x1 				//< Temperature Data Available
/// @}

/// @name Register Addresses
/// @{
#define HTS221_WHO_AM_I 				0x0F
#define HTS221_AV_CONF 					0x10
#define HTS221_CTRL_REG1 				0x20
#define HTS221_CTRL_REG2 				0x21
#define HTS221_CTRL_REG3 				0x22
#define HTS221_STATUS_REG 				0x27
#define HTS221_HUMIDITY_OUT_L 			0x28
#define HTS221_HUMIDITY_OUT_H 			0x29
#define HTS221_TEMP_OUT_L 				0x2A
#define HTS221_TEMP_OUT_H 				0x2B
#define HTS221_H0_RH_X2 				0x30
#define HTS221_H1_RH_X2 				0x31
#define HTS221_T0_DEGC_X8 				0x32
#define HTS221_T1_DEGC_X8 				0x33
#define HTS221_T1_T0_MSB 				0x35
#define HTS221_H0_T0_OUT_L 				0x36
#define HTS221_H0_T0_OUT_H 				0x37
#define HTS221_H1_T0_OUT_L 				0x3A
#define HTS221_H1_T0_OUT_H 				0x3B
#define HTS221_T0_OUT_L 				0x3C
#define HTS221_T0_OUT_H 				0x3D
#define HTS221_T1_OUT_L 				0x3E
#define HTS221_T1_OUT_H 				0x3F
/// @}

bool HTS221_Init (I2C_HandleTypeDef * hi2c);
void HTS221_ConfigDevice (uint8_t Config, uint8_t REGS1, uint8_t REGS2);
double HTS221_GetTemperature (void);
double HTS221_GetHumidity (void);

#endif /* HTS221_H_ */




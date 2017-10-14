/*
 * HTS221.c
 *
 *  Created on: Oct 14, 2017
 *      Author: pt
 */

#include "main.h"
#include "HTS221.h"

#define readI2c HTS221_I2C_Read_return

struct {
    uint8_t _H0_rH_x2;
    uint8_t _H1_rH_x2;
    uint16_t _T0_degC_x8;
	uint16_t _T1_degC_x8;
    int16_t _H0_T0_OUT;
	int16_t _H1_T0_OUT;
    int16_t _T0_OUT;
	int16_t _T1_OUT;
}HTS221_CoefStruc;

I2C_HandleTypeDef * HTS221_I2CHander;

static uint8_t HTS221_I2C_Read_return (uint8_t MemAddress);
static void HTS221_I2C_Read (uint8_t MemAddress, uint8_t* dat, uint8_t items);
static void HTS221_I2C_Write(uint8_t MemAddress, uint8_t* dat, uint8_t items);

static bool HTS221_CheckAvailable (void);
static void HTS221_EnableDevice (void);
static void HTS221_GetCoefficient (void);


static uint8_t HTS221_I2C_Read_return (uint8_t MemAddress){
	uint8_t dat = 0;
	HTS221_I2C_Read(MemAddress,&dat,1);
	return dat;
}

static void HTS221_I2C_Read (uint8_t MemAddress, uint8_t* dat, uint8_t items){
	HAL_I2C_Mem_Read(HTS221_I2CHander,HTS221_DevAddr, MemAddress, I2C_MEMADD_SIZE_8BIT,dat,items,items*2);
}

static void HTS221_I2C_Write(uint8_t MemAddress, uint8_t* dat, uint8_t items){
	HAL_I2C_Mem_Write(HTS221_I2CHander,HTS221_DevAddr, MemAddress, I2C_MEMADD_SIZE_8BIT,dat,items,items*2);
}

bool HTS221_Init (I2C_HandleTypeDef * hi2c)
{
	HTS221_I2CHander = hi2c;
	if (HTS221_CheckAvailable()){
		HTS221_EnableDevice();
		HTS221_GetCoefficient();
		HTS221_ConfigDevice(HTS221_DEFAULTSETTING, HTS221_DEFAULTREG1, HTS221_DEFAULTREG2);
		return (true);
	}
	else{
		return (false);
	}
}

static bool HTS221_CheckAvailable (void){
	if (HTS221_I2C_Read_return(HTS221_WHO_AM_I)==HTS221_DEVICE_ID)
	return (true);
	else
	return (false);
}

static void HTS221_GetCoefficient (void){
	uint8_t data = 0;
	HTS221_CoefStruc._H0_rH_x2 = HTS221_I2C_Read_return(HTS221_H0_RH_X2);
	HTS221_CoefStruc._H1_rH_x2 = HTS221_I2C_Read_return(HTS221_H1_RH_X2);
	data = HTS221_I2C_Read_return(HTS221_T1_T0_MSB);
	HTS221_CoefStruc._T0_degC_x8  = ( data & 0x3 ) << 8;
	HTS221_CoefStruc._T0_degC_x8 |= HTS221_I2C_Read_return(HTS221_T0_DEGC_X8);
	HTS221_CoefStruc._T1_degC_x8  = ( data & 0xC ) << 6;
	HTS221_CoefStruc._T1_degC_x8 |= HTS221_I2C_Read_return(HTS221_T1_DEGC_X8);
	HTS221_CoefStruc._H0_T0_OUT  = HTS221_I2C_Read_return(HTS221_H0_T0_OUT_H) << 8;
	HTS221_CoefStruc._H0_T0_OUT |= HTS221_I2C_Read_return(HTS221_H0_T0_OUT_L);
	HTS221_CoefStruc._H1_T0_OUT  = HTS221_I2C_Read_return(HTS221_H1_T0_OUT_H) << 8;
	HTS221_CoefStruc._H1_T0_OUT |= HTS221_I2C_Read_return(HTS221_H1_T0_OUT_L);
	HTS221_CoefStruc._T0_OUT  = HTS221_I2C_Read_return(HTS221_T0_OUT_H) << 8;
	HTS221_CoefStruc._T0_OUT |= HTS221_I2C_Read_return(HTS221_T0_OUT_L);
	HTS221_CoefStruc._T1_OUT  = HTS221_I2C_Read_return(HTS221_T1_OUT_H) << 8;
	HTS221_CoefStruc._T1_OUT |= HTS221_I2C_Read_return(HTS221_T1_OUT_L);
}

static void HTS221_EnableDevice (void){
	uint8_t dat = 0;
	dat |= HTS221_PD;
	dat |= HTS221_ODR_1HZ;
	HAL_I2C_Mem_Write(HTS221_I2CHander,HTS221_DevAddr,HTS221_CTRL_REG1,I2C_MEMADD_SIZE_8BIT,&dat,1,10);
}

void HTS221_ConfigDevice (uint8_t Config, uint8_t REGS1, uint8_t REGS2){
//	  uint8_t data = Config;
//	  data |= HTS221_AVGH_32;
//	  data |= HTS221_AVGT_16;
	  HTS221_I2C_Write(HTS221_CTRL_REG1,&REGS1,sizeof(REGS1));
	  HTS221_I2C_Write(HTS221_CTRL_REG2,&REGS2,sizeof(REGS2));
	  HTS221_I2C_Write(HTS221_AV_CONF,&Config,sizeof(Config));
}



double HTS221_GetHumidity (void){
	uint8_t data;
	int16_t H_OUT = 0;
	double t_H0_rH, t_H1_rH;
	double humidity = 0.0;

	data = readI2c(HTS221_STATUS_REG);
	if ( data & HTS221_H_DA ) {
	    H_OUT  = HTS221_I2C_Read_return(HTS221_HUMIDITY_OUT_H) << 8;
	    H_OUT |= HTS221_I2C_Read_return(HTS221_HUMIDITY_OUT_L);
	    t_H0_rH = HTS221_CoefStruc._H0_rH_x2 / 2.0;
	    t_H1_rH = HTS221_CoefStruc._H1_rH_x2 / 2.0;
	    humidity = t_H0_rH + ( t_H1_rH - t_H0_rH ) * ( H_OUT - HTS221_CoefStruc._H0_T0_OUT ) / ( HTS221_CoefStruc._H1_T0_OUT - HTS221_CoefStruc._H0_T0_OUT );
	  }
	  return humidity;
}

double HTS221_GetTemperature (void){
	  uint8_t data = 0;
	  uint16_t T_OUT = 0;
	  double t_T0_degC, t_T1_degC;
	  double temperature = 0.0;

	  data = readI2c(HTS221_STATUS_REG);
	  if ( data & HTS221_T_DA ) {
	    T_OUT  = readI2c(HTS221_TEMP_OUT_H) << 8;
	    T_OUT |= readI2c(HTS221_TEMP_OUT_L);
	    t_T0_degC = HTS221_CoefStruc._T0_degC_x8 / 8.0;
	    t_T1_degC = HTS221_CoefStruc._T1_degC_x8 / 8.0;
	    temperature = t_T0_degC + ( t_T1_degC - t_T0_degC ) * ( T_OUT - HTS221_CoefStruc._T0_OUT ) / ( HTS221_CoefStruc._T1_OUT - HTS221_CoefStruc._T0_OUT );
	  }
	  return temperature;
}







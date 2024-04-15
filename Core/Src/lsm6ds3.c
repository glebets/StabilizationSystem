/*
 * lsm6ds3.c
 *
 *  Created on: Mar 8, 2022
 *      Author: Глеб
 */

#include "lsm6ds3.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
//--------------------------------------------
extern I2C_HandleTypeDef hi2c3;
uint8_t buf2[14]={0};
char str1[30]={0};
//--------------------------------------------
void Error(void)
{
	LD2_OFF;
}
//--------------------------------------------
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : I2Cx_ReadData(), I2Cx_WriteData() - функции,
  * обеспечивающие взаимодействие системы с шиной i2c
  ******************************************************************************
  */
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(&hi2c3, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
	if(status != HAL_OK) Error();
	return value;
}
//--------------------------------------------
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
	if(status != HAL_OK) Error();
}
//--------------------------------------------
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : Accel_IO_Read(), Accel_IO_Write() - обёртки функций
  * I2Cx_ReadData(), I2Cx_WriteData(), повышающие читаемость кода
  ******************************************************************************
  */
uint8_t Accel_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr)
{
	return I2Cx_ReadData(DeviceAddr, RegisterAddr);
}
//--------------------------------------------
void Accel_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value)
{
	I2Cx_WriteData(DeviceAddr, RegisterAddr, Value);
}
//--------------------------------------------
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : getzgyr() - функция, получающая конечные отфильтрованные
  * данные угловой скорости по оси z и нормализующая их
  ******************************************************************************
  */
int16_t getzgyr(){
	uint8_t buf[2];
	buf[0]=Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_OUTZ_L_G);
	buf[1]=Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_OUTZ_H_G);
	int16_t res = (int16_t)((uint16_t)buf[1]<<8)+buf[0];
	filterg(&res);
	return ((res+200)/134);
}
//--------------------------------------------
uint8_t Accel_ReadID(void)
{
	uint8_t ctrl = 0x00;
	ctrl = Accel_IO_Read(0xD6,0x0F);
	return ctrl;
}
//--------------------------------------------
void AccelGyro_Read(void)
{
	sprintf(str1,"X:%06d Z:%06d\r\n", getxacc(), getzgyr());
	CDC_Transmit_FS((unsigned char*)str1, strlen(str1));
}
//--------------------------------------------
void AccInit(uint16_t InitStruct)
{
	uint8_t value = 0;
	//
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C);
	value&=~LSM6DS3_ACC_GYRO_IF_INC_MASK;
	value|=LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C,value);
	//установим бит BDU
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C);
	value&=~LSM6DS3_ACC_GYRO_BDU_MASK;
	value|=LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C,value);
	//выбор режима FIFO
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_FIFO_CTRL5);
	value&=~LSM6DS3_ACC_GYRO_FIFO_MODE_MASK;
	value|=LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_FIFO_CTRL5,value);
	//пока выключим датчик (ODR_XL = 0000)
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL1_XL);
	value&=~LSM6DS3_ACC_GYRO_ODR_XL_MASK;
	value|=LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL1_XL,value);
	//Full scale selection 2G
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL1_XL);
	value&=~LSM6DS3_ACC_GYRO_FS_XL_MASK;
	value|=LSM6DS3_ACC_GYRO_FS_XL_2g;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL1_XL,value);
	//Включим оси
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL9_XL);
	value&=~(LSM6DS3_ACC_GYRO_XEN_XL_MASK|\
					 LSM6DS3_ACC_GYRO_YEN_XL_MASK|\
					 LSM6DS3_ACC_GYRO_ZEN_XL_MASK);
	value|=(LSM6DS3_ACC_GYRO_XEN_XL_ENABLED|\
					LSM6DS3_ACC_GYRO_YEN_XL_ENABLED|\
					LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED);
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL9_XL,value);
	//Включим Data Rate 104 Гц
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL1_XL);
	value&=~LSM6DS3_ACC_GYRO_ODR_XL_MASK;
	value|=LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL1_XL,value);
}
//--------------------------------------------
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : GyroInit() - функция, инициализирующая работу гироскопа
  ******************************************************************************
  */
void GyroInit(uint16_t InitStruct)
{
	uint8_t value = 0;
	//
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C);
	value&=~LSM6DS3_ACC_GYRO_IF_INC_MASK;
	value|=LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C,value);
	//установим бит BDU
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C);
	value&=~LSM6DS3_ACC_GYRO_BDU_MASK;
	value|=LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL3_C,value);
	//выбор режима FIFO
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_FIFO_CTRL5);
	value&=~LSM6DS3_ACC_GYRO_FIFO_MODE_MASK;
	value|=LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_FIFO_CTRL5,value);
	//пока выключим датчик (ODR_G = 0000)
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL2_G);
	value&=~LSM6DS3_ACC_GYRO_ODR_G_MASK;
	value|=LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL2_G,value);
	//Выбор чувствительности датчика
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL2_G);
	value&=~LSM6DS3_ACC_GYRO_FS_G_245dps;
	value|=LSM6DS3_ACC_GYRO_FS_XL_2g;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL2_G,value);
	//Включим оси
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL10_C);
	value&=~(LSM6DS3_ACC_GYRO_XEN_G_MASK|\
					 LSM6DS3_ACC_GYRO_YEN_G_MASK|\
					 LSM6DS3_ACC_GYRO_ZEN_G_MASK);
	value|=(LSM6DS3_ACC_GYRO_XEN_G_ENABLED|\
					LSM6DS3_ACC_GYRO_YEN_G_ENABLED|\
					LSM6DS3_ACC_GYRO_ZEN_G_ENABLED);
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL10_C,value);
	//Выбор частоты опроса гироскопа 208 Гц
	value = Accel_IO_Read(0xD6,LSM6DS3_ACC_GYRO_CTRL2_G);
	value&=~LSM6DS3_ACC_GYRO_ODR_G_MASK;
	value|=LSM6DS3_ACC_GYRO_ODR_G_208Hz;
	Accel_IO_Write(0xD6,LSM6DS3_ACC_GYRO_CTRL2_G,value);
}
//--------------------------------------------
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : Accel_Gyro_Ini() - обёртка функции GyroInit(), повышающая
  * читаемость кода
  ******************************************************************************
  */
void Accel_Gyro_Ini(void)
{
	uint16_t ctrl = 0x0000;
	HAL_Delay(1000);
	GyroInit(ctrl);
}
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : check() - функция, нормализующая значение поправки
  ******************************************************************************
  */
int16_t check(int16_t input){
	return (input>-30?(input<30?input:30):-30);
}
//--------------------------------------------
//--------------------------------------------
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : correction() - функция, возвращающая значение поправки,
  * на которое необходимо изменить управляющий сигнал
  ******************************************************************************
  */
int16_t correction(float input, float setpoint, float kp, float ki, float kd, TIM_HandleTypeDef* htim){
	float dt = (float)htim->Instance->CNT * 0.0001;			//Замер времени между вызовами функции в секундах
	htim->Instance->CNT = 0;								//Обнуление таймера
	float err = setpoint - input;							//Вычисление отклонения значения от ценлевого
	static float integral = 0, prevErr = 0;
	integral = check(integral + (float)err * dt * ki);
	float D = (err - prevErr) / dt;
	prevErr = err;
	return check((int16_t)(err * kp + integral + D * kd));
}
//--------------------------------------------
/**
  ******************************************************************************
  * @file           : lsm6ds3.c
  * @brief          : filterg() - функция, обеспечивающая фильтрацию показаний
  * гироскопа.
  ******************************************************************************
  */
void filterg(int16_t* in){
	static int16_t temp = 0;
	temp += *in;
	temp /= 2;
	*in = temp;
}

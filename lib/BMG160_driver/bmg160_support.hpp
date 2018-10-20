#ifndef __BMG160_SUPPORT_H__
#define __BMG160_SUPPORT_H__

#include <Arduino.h>
#include <bmg160.hpp>

#define GYRO_CS 3

/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMG160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMG160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read */
s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 I2C_routine(void);
s8 SPI_routine(void);

/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMG160_delay_msek(u32 msek);
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmg160_data_readout_template(void);
void bmg160_init_gyro(void);
bmg160_data_t bmg160_read_gyro(void);

/*---------------------------------------------------------------------------*
*  struct bmg160_t parameters can be accessed by using bmg160
 *	bmg160_t having the following parameters
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Burst read function pointer: BMG160_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*-------------------------------------------------------------------------*/


#endif

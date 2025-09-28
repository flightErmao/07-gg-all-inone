/*!
 * @section LICENSE
 * (C) Copyright 2017 GOERTEK INC. All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename spl06.h
 * @date     "Wed NOV 1 10:00:00 2017"
 * @id       "none"
 * @version  1.0.0
 *
 * @brief
 * The core code of SPL06 device driver
 *
 * @detail
 * This file implements the core code of SPL06 device driver,
 * which includes hardware related functions, input device register,
 * device attribute files, etc.
 * This file calls some functions defined in SPL06.c and could be
 * called by spl06_i2c.c and spl06_spi.c separately.
 * REVISION: V1.0.0
*/
/****************************************************************************/
#ifndef __SPL06_H__
#define __SPL06_H__

#include <stdint.h>

#define SPL06_ENABLE_FLOAT

/*
* @brief If the user wants to support 64 bit integer calculation (needed for
* optimal pressure accuracy) please set the following definition. If
* int64 calculation is not wanted (e.g. because it would include
* large libraries), please do not set the definition.
*/
#define SPL06_ENABLE_INT64
/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
/*!
	@brief Define the calling convention of I2C bus communication routine.
	@note This includes types of parameters. This example shows the
	configuration for an I2C bus link.

    If your communication function looks like this:

    write_my_bus_xy(uint8_t device_addr, uint8_t register_addr,
    uint8_t * data, uint8_t length);

    The SPL06_WR_FUNC_PTR would equal:

	SPL06_WR_FUNC_PTR int8_t (* bus_write)(uint8_t,
    uint8_t, uint8_t *, uint8_t)

    Parameters can be mixed as needed refer to the
    refer SPL06_BUS_WRITE_FUNC  macro.
*/
/* never change this line */
#define SPL06_BUS_WRITE_FUNC(device_addr, register_addr,\
register_data, wr_len) bus_write(device_addr, register_addr,\
	register_data, wr_len)
/*!
	@brief link macro between API function calls and bus read function
	@note The bus write function can change since this is a
	system dependant issue.

    If the bus_read parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    SPL06_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_read(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a
    way that equals your definition in the
    refer SPL06_WR_FUNC_PTR definition.

    @note: this macro also includes the "MSB='1'
    for reading SPL06 addresses.
*/
/* never change this line */
#define SPL06_BUS_READ_FUNC(device_addr, register_addr,\
	register_data, rd_len)bus_read(device_addr, register_addr,\
	register_data, rd_len)

/***************************************************************/
/**\name	GET AND SET BITSLICE FUNCTIONS       */
/***************************************************************/
/* never change this line */
#define SPL06_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define SPL06_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/************************************************/
/**\name	ERROR CODES      */
/************************************************/
#define	SUCCESS					  ((uint8_t)0)
#define SPL06_NULL                (0)
#define E_SPL06_NULL_PTR         ((int8_t)-127)
#define E_SPL06_COMM_RES         ((int8_t)-1)
#define E_SPL06_OUT_OF_RANGE     ((int8_t)-2)
#define ERROR                     ((int8_t)-1)
/************************************************/
/**\name	I2C ADDRESS DEFINITION       */
/***********************************************/
/* 7-bit addr: 0x76 (SDO connected to GND); 0x77 (SDO connected to VDDIO) */
#define SPL06_I2C_ADDRESS1                  (0x76)
#define SPL06_I2C_ADDRESS2                  (0x77)
/************************************************/
/**\name	POWER MODE DEFINITION       */
/***********************************************/
#define SPL06_SLEEP_MODE                   	(0x00)
#define SPL06_NORMAL_MODE                  	(0x07)
#define SPL06_SOFT_RESET_CODE               (0x09)
/************************************************/
/**\name	SAMPLE RATE DEFINITION       */
/***********************************************/
#define SPL06_SAMPLERATE_1               	(0x00)
#define SPL06_SAMPLERATE_2                  (0x01)
#define SPL06_SAMPLERATE_4                 	(0x02)
#define SPL06_SAMPLERATE_8                 	(0x03)
#define SPL06_SAMPLERATE_16                	(0x04)
#define SPL06_SAMPLERATE_32                	(0x05)
#define SPL06_SAMPLERATE_64                	(0x06)
#define SPL06_SAMPLERATE_128               	(0x07)

/************************************************/
/**\name	OVERSAMPLING DEFINITION       */
/***********************************************/
#define SPL06_OVERSAMP_1X          		(0x00)
#define SPL06_OVERSAMP_2X               (0x01)
#define SPL06_OVERSAMP_4X               (0x02)
#define SPL06_OVERSAMP_8X               (0x03)
#define SPL06_OVERSAMP_16X              (0x04)
#define SPL06_OVERSAMP_32X              (0x05)
#define SPL06_OVERSAMP_64X              (0x06)
#define SPL06_OVERSAMP_128X             (0x07)

/************************************************/
/**\name	WORKING MODE DEFINITION       */
/***********************************************/
#define SPL06_LOW_POWER_MODE	            (0x00)
#define SPL06_STANDARD_RESOLUTION_MODE      (0x01)
#define SPL06_HIGH_RESOLUTION_MODE          (0x02)

#define SPL06_LOWPOWER_SAMPLERATE_PRESSURE	           		SPL06_SAMPLERATE_1
#define SPL06_LOWPOWER_OVERSAMP_PRESSURE	           		SPL06_OVERSAMP_2X
#define SPL06_LOWPOWER_SAMPLERATE_TEMPERATURE	       		SPL06_SAMPLERATE_1
#define SPL06_LOWPOWER_OVERSAMP_TEMPERATURE	           		SPL06_OVERSAMP_1X

#define SPL06_STANDARDRESOLUTION_SAMPLERATE_PRESSURE     	SPL06_SAMPLERATE_1
#define SPL06_STANDARDRESOLUTION_OVERSAMP_PRESSURE     		SPL06_OVERSAMP_16X
#define SPL06_STANDARDRESOLUTION_SAMPLERATE_TEMPERATURE  	SPL06_SAMPLERATE_1
#define SPL06_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE  		SPL06_OVERSAMP_1X

#define SPL06_HIGHRESOLUTION_SAMPLERATE_PRESSURE         	SPL06_SAMPLERATE_4
#define SPL06_HIGHRESOLUTION_OVERSAMP_PRESSURE         		SPL06_OVERSAMP_64X
#define SPL06_HIGHRESOLUTION_SAMPLERATE_TEMPERATURE      	SPL06_SAMPLERATE_4
#define SPL06_HIGHRESOLUTION_OVERSAMP_TEMPERATURE      		SPL06_OVERSAMP_1X

#define SPL06_PRESSURE_SHIFT 				0x04
#define SPL06_TEMPERATURE_SHIFT     		0x08

#define SPL06_TMP_SOURCE_INT				0x00
#define SPL06_TMP_SOURCE_EXT				0x80

/************************************************/
/**\name	CALIBRATION PARAMETERS DEFINITION       */
/***********************************************/
/*calibration parameters */
/* calibration data */
#define SPL06_CALIBRATION_DATA_START       	(0x10)
#define SPL06_CALIBRATION_DATA_LENGTH		(18)

/************************************************/
/**\name	REGISTER ADDRESS DEFINITION       */
/***********************************************/
#define SPL06_CHIP_ID_REG                   (0x0D)  /*Chip ID Register */
#define SPL06_RESET_REG                     (0x0C)  /*Softreset Register */
#define SPL06_INT_STATUS_REG                (0x0A)  /*Status Register */
#define SPL06_FIFO_STATUS_REG               (0x0B)  /*Status Register */
#define SPL06_PRS_CFG_REG                   (0x06)  /*Pressure Config Register */
#define SPL06_TMP_CFG_REG                   (0x07)  /*Temperature Config Register */
#define SPL06_CTRL_MEAS_REG                 (0x08)  /*Ctrl Measure Register */
#define SPL06_CONFIG_REG                    (0x09)  /*Configuration Register */

/* data */
#define SPL06_PRESSURE_MSB_REG              (0x00)  /*Pressure MSB Register */
#define SPL06_PRESSURE_LSB_REG              (0x01)  /*Pressure LSB Register */
#define SPL06_PRESSURE_XLSB_REG             (0x02)  /*Pressure XLSB Register */
#define SPL06_TEMPERATURE_MSB_REG           (0x03)  /*Temperature MSB Reg */
#define SPL06_TEMPERATURE_LSB_REG           (0x04)  /*Temperature LSB Reg */
#define SPL06_TEMPERATURE_XLSB_REG          (0x05)  /*Temperature XLSB Reg */
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR PRESSURE OVERSAMPLING */
/***********************************************/
#define SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE__POS             (0)
#define SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE__MSK             (0x0F)
#define SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE__LEN             (4)
#define SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE__REG             \
(SPL06_PRS_CFG_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR TEMPERATURE OVERSAMPLING */
/***********************************************/
#define SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE__POS             (0)
#define SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE__MSK             (0x07)
#define SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE__LEN             (3)
#define SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE__REG             \
(SPL06_TMP_CFG_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR PRESSURE SAMPLERATE */
/***********************************************/
#define SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE__POS             (4)
#define SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE__MSK             (0x70)
#define SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE__LEN             (3)
#define SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE__REG             \
(SPL06_PRS_CFG_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR TEMPERATURE SAMPLERATE */
/***********************************************/
#define SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE__POS             (4)
#define SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE__MSK             (0x70)
#define SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE__LEN             (3)
#define SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE__REG             \
(SPL06_TMP_CFG_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR POWER MODE */
/***********************************************/
#define SPL06_CTRL_MEAS_REG_POWER_MODE__POS              (0)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__MSK              (0x07)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__LEN              (3)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__REG              (SPL06_CTRL_MEAS_REG)
/************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS */
/***********************************************/
#define SPL06_WR_FUNC_PTR\
	int8_t (*bus_write)(uint8_t, uint8_t,\
			uint8_t *, uint8_t)

#define SPL06_RD_FUNC_PTR\
	int8_t (*bus_read)(uint8_t, uint8_t,\
			uint8_t *, uint8_t)

#define SPL06_MDELAY_DATA_TYPE uint16_t
/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/
/*!
 * @brief This structure holds all device specific calibration parameters
 */
struct spl06_calib_param_t {	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
};

/*!
 * @brief This structure holds SPL06 initialization parameters
 */
struct spl06_t {
	struct spl06_calib_param_t calib_param;/**<calibration data*/

	uint8_t chip_id;/**< chip id of the sensor*/
	uint8_t dev_addr;/**< device address of the sensor*/
	uint32_t i32kP; /*!pressure scale factor variable */
	uint32_t i32kT; /*!temperature scale factor variable */
  	uint8_t oversamp_pressure;/*!pressure oversampling variable */	
  	uint8_t oversamp_temperature;/*!temperature oversampling variable */	
	uint8_t samplerate_pressure;/*!pressure sampling rate */
	uint8_t samplerate_temperature;/*!temperature sampling rate */
	
	SPL06_WR_FUNC_PTR;/**< bus write function pointer*/
	SPL06_RD_FUNC_PTR;/**< bus read function pointer*/
	void (*delay_msec)(SPL06_MDELAY_DATA_TYPE);/**< delay function pointer*/	
};

/**************************************************************/
/**\name	FUNCTION FOR  INTIALIZATION                       */
/**************************************************************/
/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the SPL06 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	@param *spl06 structure pointer.
 *
 *	@note While changing the parameter of the p_spl06
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_init(struct spl06_t *spl06);
/**************************************************************/
/**\name	FUNCTION FOR READ UNCOMPENSATED TEMPERATURE     */
/**************************************************************/
/*!
 *	@brief This API is used to read unco3mpensated temperature
 *	in the registers 0x03, 0x04 and 0x05
 *	@note 0x03 -> MSB -> bit from 0 to 7
 *	@note 0x04 -> LSB -> bit from 0 to 7
 *	@note 0x05 -> XLSB -> bit from 0 to 7
 *
 *	@param temperature : The uncompensated temperature.

 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_read_raw_temperature(int32_t *temperature);
/**************************************************************/
/**\name	FUNCTION FOR READ TRUE TEMPERATURE S32 OUTPUT    */
/**************************************************************/
/*!
 *	@brief Reads actual temperature
 *	from uncompensated temperature
 *	@note Returns the value in 0.01 degree Centigrade
 *	@note Output value of "5123" equals 51.23 DegC.
 *
 *  @param none
 *
 *  @return actual temperature output as int32_t
 *
*/
int32_t spl06_compensate_temperature_int32(void);
/**************************************************************/
/**\name	FUNCTION FOR READ UNCOMPENSATED PRESSURE     */
/**************************************************************/
/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0x00, 0x01 and 0x02
 *	@note 0x00 -> MSB -> bit from 0 to 7
 *	@note 0x01 -> LSB -> bit from 0 to 7
 *	@note 0x02 -> XLSB -> bit from 0 to 7
 *
 *	@param pressure : The value of uncompensated pressure
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_read_raw_pressure(int32_t *pressure);
/**************************************************************/
/**\name	FUNCTION FOR READ TRUE PRESSURE S32 OUTPUT    */
/**************************************************************/
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *  @param  none
 *
 *  @return Returns the Actual pressure out put as int32_t
 *
*/
int32_t spl06_compensate_pressure_int32(void);
/**************************************************************/
/**\name	FUNCTION FOR READ TRUE TEMPERATURE AND PRESSURE    */
/**************************************************************/
/*!
 * @brief This API reads the true pressure and temperature
 *
 *  @param  pressure : The value of compensated pressure.
 *  @param  temperature : The value of compensated temperature.
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_read_pressure_temperature(int32_t *pressure, int32_t *temperature);
/**************************************************************/
/**\name	FUNCTION FOR READ CALIBRATION DATA    */
/**************************************************************/
/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address
 *------------|----------------------------------
 *	c0    |  0x10 and 0x11 
 *	c1    |  0x11 and 0x12  
 *	c00    |  0x13 and 0x14 and 0x15 
 *	c10    |  0x16 and 0x17  
 *	c01    |  0x18 and 0x19 
 *	c11    |  0x1A and 0x1B  
 *	c20    |  0x1C and 0x1D  
 *	c21    |  0x1E and 0x1F 
 *	c30    |  0x20 and 0x21  
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_get_calib_param(void);
/**************************************************************/
/**\name	FUNCTION FOR OVERSAMPLING TEMPERATURE AND PRESSURE    */
/**************************************************************/
/*!
 *	@brief This API is used to get
 *	the temperature oversampling setting in the register 0x07
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_OVERSAMP_1X
 *       0x01               |  SPL06_OVERSAMP_2X
 *       0x02               |  SPL06_OVERSAMP_4X
 *       0x03               |  SPL06_OVERSAMP_8X
 *       0x04               |  SPL06_OVERSAMP_16X
 *       0x05               |  SPL06_OVERSAMP_32X
 *       0x06               |  SPL06_OVERSAMP_64X
 *       0x07               |  SPL06_OVERSAMP128X
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_get_oversamp_temperature(uint8_t *v_value_u8);
/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0x07
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_OVERSAMP_1X
 *       0x01               |  SPL06_OVERSAMP_2X
 *       0x02               |  SPL06_OVERSAMP_4X
 *       0x03               |  SPL06_OVERSAMP_8X
 *       0x04               |  SPL06_OVERSAMP_16X
 *       0x05               |  SPL06_OVERSAMP_32X
 *       0x06               |  SPL06_OVERSAMP_64X
 *       0x07               |  SPL06_OVERSAMP_128X 
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_set_oversamp_temperature(uint8_t v_value_u8);
/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting in the register 0x06
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_OVERSAMP_1X
 *       0x01               |  SPL06_OVERSAMP_2X
 *       0x02               |  SPL06_OVERSAMP_4X
 *       0x03               |  SPL06_OVERSAMP_8X
 *       0x04               |  SPL06_OVERSAMP_16X
 *       0x05               |  SPL06_OVERSAMP_32X
 *       0x06               |  SPL06_OVERSAMP_64X
 *       0x07               |  SPL06_OVERSAMP_128X 
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_get_oversamp_pressure(uint8_t *v_value_u8);
/*!
 *	@brief This API is used to set
 *	the pressure oversampling setting in the register 0x06
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_OVERSAMP_1X
 *       0x01               |  SPL06_OVERSAMP_2X
 *       0x02               |  SPL06_OVERSAMP_4X
 *       0x03               |  SPL06_OVERSAMP_8X
 *       0x04               |  SPL06_OVERSAMP_16X
 *       0x05               |  SPL06_OVERSAMP_32X
 *       0x06               |  SPL06_OVERSAMP_64X
 *       0x07               |  SPL06_OVERSAMP_128X 
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_set_oversamp_pressure(uint8_t v_value_u8);
/**************************************************************/
/**\name	FUNCTION FOR SAMPLERATE TEMPERATURE AND PRESSURE	  */
/**************************************************************/
/*!
 *	@brief This API is used to get
 *	the pressure samplerate setting in the register 0x07
 *	bits from 0 to 2
 *
 *        value             | Temperature samplerate
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_SAMPLERATE_1
 *       0x01               |  SPL06_SAMPLERATE_2
 *       0x02               |  SPL06_SAMPLERATE_4
 *       0x03               |  SPL06_SAMPLERATE_8
 *       0x04               |  SPL06_SAMPLERATE_16
 *       0x05               |  SPL06_SAMPLERATE_32
 *       0x06               |  SPL06_SAMPLERATE_64
 *       0x07               |  SPL06_SAMPLERATE_128
 *
 *  @param  v_value_u8 : The value of pressure sample rate
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_get_samplerate_temperature(uint8_t *v_value_u8);
/*!
 *	@brief This API is used to set
 *	the temperature sample rate setting in the register 0x07
 *	bits from 0 to 2
 *
 *        value             | Temperature samplerate
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_SAMPLERATE_1
 *       0x01               |  SPL06_SAMPLERATE_2
 *       0x02               |  SPL06_SAMPLERATE_4
 *       0x03               |  SPL06_SAMPLERATE_8
 *       0x04               |  SPL06_SAMPLERATE_16
 *       0x05               |  SPL06_SAMPLERATE_32
 *       0x06               |  SPL06_SAMPLERATE_64
 *       0x07               |  SPL06_SAMPLERATE_128
 *
 *  @param  v_value_u8 : The value of temperature sample rate
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_set_samplerate_temperature(uint8_t v_value_u8);
/*!
 *	@brief This API is used to get
 *	the pressure sample rate setting in the register 0x06
 *	bits from 4 to 6
 *
 *        value             | Pressure samplerate
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_SAMPLERATE_1
 *       0x01               |  SPL06_SAMPLERATE_2
 *       0x02               |  SPL06_SAMPLERATE_4
 *       0x03               |  SPL06_SAMPLERATE_8
 *       0x04               |  SPL06_SAMPLERATE_16
 *       0x05               |  SPL06_SAMPLERATE_32
 *       0x06               |  SPL06_SAMPLERATE_64
 *       0x07               |  SPL06_SAMPLERATE_128
 *
 *  @param  v_value_u8 : The value of pressure sample rate
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_get_samplerate_pressure(uint8_t *v_value_u8);
/*!
 *	@brief This API is used to set
 *	the pressure sample rate setting in the register 0x06
 *	bits from 4 to 6
 *
 *        value             | Pressure samplerate
 *  ------------------------|------------------------------
 *       0x00               |  SPL06_SAMPLERATE_1
 *       0x01               |  SPL06_SAMPLERATE_2
 *       0x02               |  SPL06_SAMPLERATE_4
 *       0x03               |  SPL06_SAMPLERATE_8
 *       0x04               |  SPL06_SAMPLERATE_16
 *       0x05               |  SPL06_SAMPLERATE_32
 *       0x06               |  SPL06_SAMPLERATE_64
 *       0x07               |  SPL06_SAMPLERATE_128
 *
 *  @param  v_value_u8 : The value of pressure sample rate
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_set_samplerate_pressure(uint8_t v_value_u8);
/**************************************************************/
/**\name	FUNCTION FOR POWER MODE    */
/**************************************************************/
/*!
 *	@brief This API used to get the
 *	Operational Mode from the sensor in the register 0x08 bit 0 to 2
 *
 *	@param power_mode : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | SPL06_SLEEP_MODE
 *	0x07             | SPL06_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_get_power_mode(uint8_t *power_mode);
/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0x08 bit 0 to 2
 *
 *	@param power_mode : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | SPL06_SLEEP_MODE
 *	0x07             | SPL06_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_set_power_mode(uint8_t power_mode);
/**************************************************************/
/**\name	FUNCTION FOR SOFT RESET   */
/**************************************************************/
/*!
 * @brief Used to reset the sensor
 * The value 0x0C is written to the
 * 0x09 register the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using spl06_set_softreset().
 *
 * @note Usage Hint : spl06_set_softreset()
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_set_soft_rst(void);
/**************************************************************/
/**\name	FUNCTION FOR WORK MODE   */
/**************************************************************/
/*!
 *	@brief This API is used to write
 *	 the working mode of the sensor
 *
 *  @param work_mode : The value of work mode
 *   value      |  mode
 * -------------|-------------
 *    0         | SPL06_LOW_POWER_MODE
 *    1         | SPL06_STANDARD_RESOLUTION_MODE
 *    2         | SPL06_HIGH_RESOLUTION_MODE
 *
 *   @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
int8_t spl06_set_work_mode(uint8_t work_mode);
/**************************************************************/
/**\name	FUNCTION FOR COMMON READ AND WRITE    */
/**************************************************************/
/*!
 * @brief
 *	This API write the data to
 *	the given register
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
int8_t spl06_write_register(uint8_t v_addr_u8, uint8_t *v_data_u8, uint8_t v_len_u8);
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
int8_t spl06_read_register(uint8_t v_addr_u8, uint8_t *v_data_u8, uint8_t v_len_u8);
/**************************************************************/
/**\name	FUNCTION FOR TRUE TEMPERATURE CALCULATION   */
/**************************************************************/
#ifdef SPL06_ENABLE_FLOAT
/*!
 * @brief This API used to read
 * actual temperature from uncompensated temperature
 * @note Returns the value in Degree centigrade
 * @note Output value of "51.23" equals 51.23 DegC.
 *
 *  @param none
 *
 *  @return
 *	Actual temperature in floating point
 *
*/
double spl06_compensate_temperature_double(void);
/**************************************************************/
/**\name	FUNCTION FOR TRUE PRESSURE CALCULATION   */
/**************************************************************/
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns pressure in Pa as double.
 *	@note Output value of "96386.2"
 *	equals 96386.2 Pa = 963.862 hPa.
 *
 *  @param none
 *
 *  @return
 *	Actual pressure in floating point
 *
*/
double spl06_compensate_pressure_double(void);
#endif
#if defined(SPL06_ENABLE_INT64) && defined(SPL06_64BITSUPPORT_PRESENT)
/*!
 * @brief This API used to read actual pressure from uncompensated pressure
 * @note returns the value in Pa as unsigned 32 bit
 * integer in Q24.8 format (24 integer bits and
 * 8 fractional bits). Output value of "24674867"
 * represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *  @param none
 *
 *  @return actual pressure as 64bit output
 *
*/
int64_t spl06_compensate_pressure_int64(void);
#endif
/**************************************************************/
/**\name	FUNCTION FOR DELAY CALCULATION DURING FORCEMODE  */
/**************************************************************/
/*!
 * @brief Computing measurment time for sensor data read
 *
 *  @param meastime: The value of measurment time
 *
 *  @return 0
 *
 *
 */
int8_t spl06_compute_meas_time(uint32_t *meastime);
#endif

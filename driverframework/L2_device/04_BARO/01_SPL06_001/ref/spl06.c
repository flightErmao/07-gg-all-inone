/*!
 * @section LICENSE
 * (C) Copyright 2017 GOERTEK INC. All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename spl06.c
 * @date     "Wed NOV 1 10:00:00 2017"
 * @id       "none"
 * @version  1.0.0
 *
 * @brief
 * API for accessing the SPL06 sensor

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
#include "spl06.h"
#include "asm/div64.h"
#include <linux/math64.h>
static struct spl06_t *p_spl06; /**< pointer to SPL06 */

/*!
 * @brief list all the coefficients parameter
*/
static const u32 scalefactor_list[] = {
	524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};
/*!
 * @brief list all the measurment time
 * that could be set[unit:0.1ms]
*/
static const u32 meastime_list[] = {
	36, 52, 84, 148, 276, 532, 1044, 2068};
/*!
 * @brief list all the sample rate
 * that could be set[unit:Hz]
*/
static const u32 samplerate_list[] = {
	1, 2, 4, 8, 16, 32, 64, 128};

/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *   and assign the chip id and I2C address of the SPL06 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *	@param *spl06 structure pointer.
 *	@note While changing the parameter of the p_spl06
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
s8 spl06_init(struct spl06_t *spl06)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;

	p_spl06 = spl06;/* assign SPL06 ptr */
	/* read chip id */
	com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
	p_spl06->dev_addr, 
	SPL06_CHIP_ID_REG, 
	&v_data_u8, 1);/* read Chip Id */
	p_spl06->chip_id = v_data_u8;
	/* readout spl06 calibparam structure */
	com_rslt += spl06_get_calib_param();
	return com_rslt;
}

/*!
 *	@brief This API is used to read uncompensated temperature
 *	in the registers 0x03, 0x04 and 0x05
 *	@note 0x03 -> MSB -> bit from 0 to 7
 *	@note 0x04 -> LSB -> bit from 0 to 7
 *	@note 0x05 -> LSB -> bit from 0 to 7
 *
 *	@param temperature : The uncompensated temperature.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
s8 spl06_read_raw_temperature(s32 *temperature)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 a_data_u8r[3] = {0};
	s32 utemperature = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* read temperature data */
			com_rslt  = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_TEMPERATURE_MSB_REG, 
			&a_data_u8r[0], 
			1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_TEMPERATURE_LSB_REG, 
			&a_data_u8r[1], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_TEMPERATURE_XLSB_REG, 
			&a_data_u8r[2], 1);

			utemperature = (s32)a_data_u8r[0]<<16 | (s32)a_data_u8r[1]<<8 | (s32)a_data_u8r[2];	 
			*temperature= (utemperature&0x800000) ? (0xFF000000|utemperature) : utemperature;
		}
	return com_rslt;
}
/*!
 *	@brief Reads actual temperature
 *	from uncompensated temperature
 *	@note Returns the value in 0.01 degree Centigrade
 *	@note Output value of "5123" equals 51.23 DegC.
 *
 *  @param none
 *
 *  @return Actual temperature output as s32
 *
*/
s32 spl06_compensate_temperature_int32(void)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	s32 utemperature = 0;/* uncompensated temperature */
	s32 temperature = 0;
	s32 fTsc;
	s32 REM;
	s64 left;

	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {	
			com_rslt = spl06_read_raw_temperature(&utemperature);
			if (com_rslt == SUCCESS) {
				/* calculate true temperature*/
				left = ((s64)utemperature<<16);
				REM = (s32)(div_s64(left, p_spl06->i32kT));
				fTsc = REM;

				/* Actual temperature should be divided by 256*/
				temperature = ((s32)p_spl06->calib_param.c0 << 7) + ((s32)((s32)p_spl06->calib_param.c1 * fTsc) >> 8);
				/* The result temperature unit should be 0.01deg */
				temperature = (temperature * 100) / 256;
			}
	}
	return temperature;
}
/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0x00, 0x01 and 0x02
 *	@note 0x00 -> MSB -> bit from 0 to 7
 *	@note 0x01 -> LSB -> bit from 0 to 7
 *	@note 0x02 -> LSB -> bit from 0 to 7
 *
 *	@param pressure : The value of uncompensated pressure
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
s8 spl06_read_raw_pressure(s32* pressure)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 a_data_u8r[3] = { 0 };
	s32 upressure = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
	}
	else {
		com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_PRESSURE_MSB_REG,
			&a_data_u8r[0], 1);
		com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_PRESSURE_LSB_REG,
			&a_data_u8r[1], 1);
		com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_PRESSURE_XLSB_REG,
			&a_data_u8r[2], 1);

		upressure = (s32)a_data_u8r[0] << 16 | (s32)a_data_u8r[1] << 8 | (s32)a_data_u8r[2];
		*pressure = (upressure & 0x800000) ? (0xFF000000 | upressure) : upressure;
	}
	return com_rslt;
}
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *  @param  none
 *
 *  @return Returns the Actual pressure out put as s32
 *
*/
s32 spl06_compensate_pressure_int32(void)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	s32 utemperature = 0, upressure = 0;/* uncompensated temperature and pressure */
	s32 pressure = 0;
	s32 fTsc, fPsc;
	s32 qua2, qua3;
	s32 rem;
	s64	left;

	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
	}
	else {
		/* update the uncompensated temperature and pressure*/
		com_rslt = spl06_read_raw_temperature(&utemperature);
		com_rslt += spl06_read_raw_pressure(&upressure);
		if (com_rslt == SUCCESS) {
			left = ((s64)utemperature << 16);
				rem = (s32)(div_s64( left , p_spl06->i32kT));
				fTsc = rem;
				left = ((s64)upressure << 16);
				rem = (s32)(div_s64(left , p_spl06->i32kP));
				fPsc = rem;
			
				qua2 = ((s32)p_spl06->calib_param.c20 << 8) + ((s32)((s32)p_spl06->calib_param.c30 * fPsc) >> 8);
				qua3 = ((s32)p_spl06->calib_param.c10 << 8) + (s32)(((s64)qua2 * (s64)fPsc) >> 16);
				pressure = ((s32)p_spl06->calib_param.c00 << 8) + (s32)(((s64)qua3 * (s64)fPsc) >> 16);
				pressure += ((s32)((s32)p_spl06->calib_param.c01 * fTsc) >> 8);

				qua2 = ((s32)p_spl06->calib_param.c11 << 8) + ((s32)((s32)p_spl06->calib_param.c21 * fPsc) >> 8);
				qua3 = (s32)(((s64)qua2 * (s64)fPsc) >> 16);	
				pressure += (s32)(((s64)qua3 * (s64)fTsc) >> 16);
				/* Actual temperature should be divided by 256*/
				pressure = (pressure >> 8);
			}
		}
	return pressure;
}

/*!
 * @brief This API reads the true pressure and temperature
 *
 *  @param  pressure : The value of compensated pressure.
 *  @param  temperature : The value of compensated temperature.
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
s8 spl06_read_pressure_temperature(s32 *pressure, s32 *temperature)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	s32 v_uncomp_pressure_s32 = 0;
	s32 v_uncomp_temperature_s32 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* read trure pressure and temperature*/
			*temperature = spl06_compensate_temperature_int32();
			*pressure = spl06_compensate_pressure_int32();
		}
	return com_rslt;
}

/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address 
 *------------|-------------------------
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
s8 spl06_get_calib_param(void)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 a_data_u8r[3] = {0};

	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+1, 
			&a_data_u8r[1], 1);
			p_spl06->calib_param.c0 = (s16)a_data_u8r[0]<<4 | a_data_u8r[1]>>4;
			p_spl06->calib_param.c0 = (p_spl06->calib_param.c0&0x0800) ? (0xF000|p_spl06->calib_param.c0) : p_spl06->calib_param.c0;

			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+1, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+2, 
			&a_data_u8r[1], 1);
			p_spl06->calib_param.c1 = (s16)(a_data_u8r[0]&0x0F)<<8 | a_data_u8r[1];
			p_spl06->calib_param.c1 = (p_spl06->calib_param.c1&0x0800) ? (0xF000|p_spl06->calib_param.c1) : p_spl06->calib_param.c1;
		
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+3, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+4, 
			&a_data_u8r[1], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+5, 
			&a_data_u8r[2], 1);
			p_spl06->calib_param.c00 = (s32)a_data_u8r[0]<<12 | (s32)a_data_u8r[1]<<4 | (s32)a_data_u8r[2]>>4;
			p_spl06->calib_param.c00 = (p_spl06->calib_param.c00&0x080000) ? (0xFFF00000|p_spl06->calib_param.c00) : p_spl06->calib_param.c00;
		
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+5, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+6, 
			&a_data_u8r[1], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+7, 
			&a_data_u8r[2], 1);
			p_spl06->calib_param.c10 = (s32)(a_data_u8r[0]&0x0F)<<16 | (s32)a_data_u8r[1]<<8 | a_data_u8r[2];
			p_spl06->calib_param.c10 = (p_spl06->calib_param.c10&0x080000) ? (0xFFF00000|p_spl06->calib_param.c10) : p_spl06->calib_param.c10;
		
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+8, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+9, 
			&a_data_u8r[1], 1);
			p_spl06->calib_param.c01 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+10, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+11, 
			&a_data_u8r[1], 1);
			p_spl06->calib_param.c11 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+12, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+13, 
			&a_data_u8r[1], 1);
			p_spl06->calib_param.c20 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+14, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+15, 
			&a_data_u8r[1], 1);
			p_spl06->calib_param.c21 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+16, 
			&a_data_u8r[0], 1);
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CALIBRATION_DATA_START+17, 
			&a_data_u8r[1], 1);
			p_spl06->calib_param.c30 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
		}
	return com_rslt;
}

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
s8 spl06_get_oversamp_temperature(u8 *v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* read temperature over sampling*/
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = SPL06_GET_BITSLICE(v_data_u8,
			SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE);
			/* assign temperature oversampling*/
			p_spl06->oversamp_temperature = *v_value_u8;
		}
	return com_rslt;
}

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
s8 spl06_set_oversamp_temperature(u8 v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				/* write over sampling*/
				v_data_u8 =
				SPL06_SET_BITSLICE(v_data_u8,
				SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE,
				 v_value_u8);
				v_data_u8 |= SPL06_TMP_SOURCE_EXT;
				com_rslt +=
				p_spl06->SPL06_BUS_WRITE_FUNC(
				p_spl06->dev_addr,
				SPL06_TMP_CFG_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8, 1);
				p_spl06->oversamp_temperature = v_value_u8;
				p_spl06->i32kT = scalefactor_list[v_value_u8];
			}
		}
		if(v_value_u8 > SPL06_OVERSAMP_8X) {		 
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
			v_data_u8 |= SPL06_TEMPERATURE_SHIFT;		 
			com_rslt += p_spl06->SPL06_BUS_WRITE_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
		} else {		 
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
			v_data_u8 &= (~SPL06_TEMPERATURE_SHIFT);		
			com_rslt += p_spl06->SPL06_BUS_WRITE_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
		} 			
	return com_rslt;
}

/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting in the register 0x06
 *	bits from 5 to 7
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
 *       0x07               |  SPL06_OVERSAMP128X
 *
 *  @param v_value_u8 :The value of pressure over sampling
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
s8 spl06_get_oversamp_pressure(u8 *v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* read pressure over sampling */
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = SPL06_GET_BITSLICE(v_data_u8,
			SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE);
			/* assign pressure oversampling*/
			p_spl06->oversamp_pressure = *v_value_u8;
		}		
	return com_rslt;
}

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
 *       0x07               |  SPL06_OVERSAMP128X
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
s8 spl06_set_oversamp_pressure(u8 v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				/* write pressure over sampling */
				v_data_u8 = SPL06_SET_BITSLICE(
				v_data_u8,
				SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE,
				v_value_u8);
				com_rslt +=
				p_spl06->SPL06_BUS_WRITE_FUNC(
				p_spl06->dev_addr,
				SPL06_PRS_CFG_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8, 1);
				p_spl06->oversamp_pressure = v_value_u8;
				p_spl06->i32kP = scalefactor_list[v_value_u8];
			}
		}
		if(v_value_u8 > SPL06_OVERSAMP_8X) {		 
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
			v_data_u8 |= SPL06_PRESSURE_SHIFT;		 
			com_rslt += p_spl06->SPL06_BUS_WRITE_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
		} else {		 
			com_rslt += p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
			v_data_u8 &= (~SPL06_PRESSURE_SHIFT);		
			com_rslt += p_spl06->SPL06_BUS_WRITE_FUNC(
			p_spl06->dev_addr, 
			SPL06_CONFIG_REG, 
			&v_data_u8, 1);
		} 			
	return com_rslt;
}

/*!
 *	@brief This API is used to get
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
s8 spl06_get_samplerate_temperature(u8 *v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return	E_SPL06_NULL_PTR;
		} else {
			/* read pressure over sampling */
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = SPL06_GET_BITSLICE(v_data_u8,
			SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE);
			/* assign temperature samplerate*/
			p_spl06->samplerate_temperature = *v_value_u8;
		}
	return com_rslt;
}

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
s8 spl06_set_samplerate_temperature(u8 v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				/* write pressure over sampling */
				v_data_u8 = SPL06_SET_BITSLICE(
				v_data_u8,
				SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE,
				v_value_u8);
				com_rslt +=
				p_spl06->SPL06_BUS_WRITE_FUNC(
				p_spl06->dev_addr,
				SPL06_TMP_CFG_REG_SAMPLERATE_TEMPERATURE__REG,
				&v_data_u8, 1);
				p_spl06->samplerate_temperature= v_value_u8;
			}
		}
	return com_rslt;
}

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
s8 spl06_get_samplerate_pressure(u8 *v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return	E_SPL06_NULL_PTR;
		} else {
			/* read pressure over sampling */
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE__REG,
			&v_data_u8, 1);
			*v_value_u8 = SPL06_GET_BITSLICE(v_data_u8,
			SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE);
			/* assign pressure samplerate*/
			p_spl06->oversamp_pressure = *v_value_u8;
		}
	return com_rslt;
}

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
s8 spl06_set_samplerate_pressure(u8 v_value_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE__REG,
			&v_data_u8, 1);
			if (com_rslt == SUCCESS) {
				/* write pressure over sampling */
				v_data_u8 = SPL06_SET_BITSLICE(
				v_data_u8,
				SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE,
				v_value_u8);
				com_rslt +=
				p_spl06->SPL06_BUS_WRITE_FUNC(
				p_spl06->dev_addr,
				SPL06_PRS_CFG_REG_SAMPLERATE_PRESSURE__REG,
				&v_data_u8, 1);
				p_spl06->samplerate_pressure = v_value_u8;
			}
		}
	return com_rslt;
}

/*!
 *	@brief This API used to get the
 *	Operational Mode from the sensor in the register 0x08 bit 0 and 2
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
s8 spl06_get_power_mode(u8 *power_mode)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_mode_u8 = 0;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* read the power mode*/
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			SPL06_CTRL_MEAS_REG_POWER_MODE__REG,
			&v_mode_u8, 1);
			*power_mode = SPL06_GET_BITSLICE(v_mode_u8,
			SPL06_CTRL_MEAS_REG_POWER_MODE);
		}
	return com_rslt;
}

/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0x08 bit 0 and 2
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
s8 spl06_set_power_mode(u8 power_mode)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;

	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			if (power_mode <= SPL06_NORMAL_MODE) {
				/* write the power mode*/
				com_rslt = p_spl06->SPL06_BUS_WRITE_FUNC(
				p_spl06->dev_addr,
				SPL06_CTRL_MEAS_REG_POWER_MODE__REG,
				&power_mode, 1);				
			} else {
			com_rslt = E_SPL06_OUT_OF_RANGE;
			}
		}
	return com_rslt;
}
/*!
 * @brief Used to reset the sensor
 * The value 0x09 is written to the 0x0C register
 * the device is reset using the
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
s8 spl06_set_soft_rst(void)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	u8 v_data_u8 = SPL06_SOFT_RESET_CODE;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* write soft reset */
			com_rslt = p_spl06->SPL06_BUS_WRITE_FUNC(
			p_spl06->dev_addr,
			SPL06_RESET_REG, 
			&v_data_u8, 1);
		}
	return com_rslt;
}

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
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
s8 spl06_set_work_mode(u8 work_mode)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;

	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
	} else {
		if (work_mode <= SPL06_HIGH_RESOLUTION_MODE) {
			switch (work_mode) {
			/* write work mode*/
			case SPL06_LOW_POWER_MODE:
				p_spl06->samplerate_pressure =
				SPL06_LOWPOWER_SAMPLERATE_PRESSURE;
				p_spl06->oversamp_pressure =
				SPL06_LOWPOWER_OVERSAMP_PRESSURE;				
				p_spl06->samplerate_temperature =
				SPL06_LOWPOWER_SAMPLERATE_TEMPERATURE;				
				p_spl06->oversamp_temperature =
				SPL06_LOWPOWER_OVERSAMP_TEMPERATURE;
				break;
			case SPL06_STANDARD_RESOLUTION_MODE:			
				p_spl06->samplerate_pressure =
				SPL06_STANDARDRESOLUTION_SAMPLERATE_PRESSURE;				
				p_spl06->oversamp_pressure =
				SPL06_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
				p_spl06->samplerate_temperature =
				SPL06_STANDARDRESOLUTION_SAMPLERATE_TEMPERATURE;
				p_spl06->oversamp_temperature =
				SPL06_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;					
				break;
			case SPL06_HIGH_RESOLUTION_MODE:			
				p_spl06->samplerate_pressure =
				SPL06_HIGHRESOLUTION_SAMPLERATE_PRESSURE;				
				p_spl06->oversamp_pressure =
				SPL06_HIGHRESOLUTION_OVERSAMP_PRESSURE;
				p_spl06->samplerate_temperature =
				SPL06_HIGHRESOLUTION_SAMPLERATE_TEMPERATURE;
				p_spl06->oversamp_temperature =
				SPL06_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;					
				break;
			}
			com_rslt = spl06_set_oversamp_pressure(p_spl06->oversamp_temperature);
			com_rslt += spl06_set_samplerate_pressure(p_spl06->samplerate_pressure);			
			com_rslt += spl06_set_samplerate_temperature(p_spl06->samplerate_temperature);
			com_rslt += spl06_set_oversamp_temperature(p_spl06->oversamp_temperature);
		} else {
		com_rslt = E_SPL06_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

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
s8 spl06_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = p_spl06->SPL06_BUS_WRITE_FUNC(
			p_spl06->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}

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
s8 spl06_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = p_spl06->SPL06_BUS_READ_FUNC(
			p_spl06->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
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
double spl06_compensate_temperature_double(void)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	s32 utemperature = 0;/* uncompensated temperature */
	double temperature = 0;
	double fTsc;
	
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			com_rslt = spl06_read_raw_temperature(&utemperature);
			if (com_rslt == SUCCESS) {
				fTsc = (double)utemperature / (double)p_spl06->i32kT;
				/* The result temperature unit should be 0.01deg */
				temperature = (p_spl06->calib_param.c0 * 0.5 + p_spl06->calib_param.c1 * fTsc) * 100.0;
			}
		}
	return temperature;
}

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
double spl06_compensate_pressure_double(void)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	s32 utemperature = 0, upressure = 0;/* uncompensated temperature and pressure */
	double fTsc, fPsc;
	double qua2, qua3;
	double pressure;	
	
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* read the uncompensated temperature and pressure*/
			com_rslt = spl06_read_raw_temperature(&utemperature);
			com_rslt += spl06_read_raw_pressure(&upressure);
			if (com_rslt == SUCCESS) {
				fTsc = (double)utemperature / (double)p_spl06->i32kT;
				fPsc = (double)upressure / (double)p_spl06->i32kP;
				qua2 = p_spl06->calib_param.c10 + fPsc * (p_spl06->calib_param.c20 + fPsc * p_spl06->calib_param.c30);
				qua3 = fTsc * fPsc * (p_spl06->calib_param.c11 + fPsc * p_spl06->calib_param.c21);

				pressure = p_spl06->calib_param.c00 + fPsc * qua2 + fTsc * p_spl06->calib_param.c01 + qua3;
			}
		}
	return pressure;
}
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
s64 spl06_compensate_pressure_int64(void)
{
	/* variable used to return communication result*/
	s8 com_rslt = ERROR;
	s32 utemperature = 0, upressure = 0;/* uncompensated temperature and pressure */
	s64 pressure = 0;
	s64 fTsc, fPsc;
	s64 qua2, qua3;
	s32 rem;
	s64	left;
	
	/* check the p_spl06 struct pointer as NULL*/
	if (p_spl06 == SPL06_NULL) {
		return  E_SPL06_NULL_PTR;
		} else {
			/* update the uncompensated temperature and pressure*/
			com_rslt = spl06_read_raw_temperature(&utemperature);
			com_rslt += spl06_read_raw_pressure(&upressure);
			if (com_rslt == SUCCESS) {
				left = (s64)utemperature << 16);
				rem = (s64)(do_div(left , p_spl06->i32kT));
				fTsc = left;
				left = (s64)upressure << 16);
				rem = (s64)(do_div(left , p_spl06->i32kP));
				fPsc = left;
			
				qua2 = ((s64)p_spl06->calib_param.c20 << 8) + ((s64)((s64)p_spl06->calib_param.c30 * fPsc) >> 8);
				qua3 = ((s64)p_spl06->calib_param.c10 << 8) + (s64)(((s64)qua2 * (s64)fPsc) >> 16);
				pressure = ((s64)p_spl06->calib_param.c00 << 8) + (s64)(((s64)qua3 * (s64)fPsc) >> 16);
				pressure += ((s64)((s64)p_spl06->calib_param.c01 * fTsc) >> 8);

				qua2 = ((s64)p_spl06->calib_param.c11 << 8) + ((s64)((s64)p_spl06->calib_param.c21 * fPsc) >> 8);
				qua3 = (s64)(((s64)qua2 * (s64)fPsc) >> 16);	
				pressure += (s64)(((s64)qua3 * (s64)fTsc) >> 16);
				/* Actual temperature should be divided by 256*/
				pressure = (pressure >> 8);
			}
		}
	return pressure;
}
#endif

/*!
 * @brief Computing measurment time for sensor data read
 *
 *  @param meastime: The value of measurment time
 *
 *  @return 0
 *
 *
 */
s8 spl06_compute_meas_time(u32 *meastime)
{
	/* variable used to return communication result*/
	s8 com_rslt = SUCCESS;

	*meastime = (samplerate_list[p_spl06->samplerate_temperature]
	 * meastime_list[p_spl06->oversamp_temperature] 
	 + samplerate_list[p_spl06->samplerate_pressure]
	 * meastime_list[p_spl06->oversamp_pressure])/10;

	return com_rslt;
}

#ifndef DPS3xx_H_INCLUDED
#define DPS3xx_H_INCLUDED

#include <stdint.h>
#include <rtdevice.h>

// #include <Arduino.h>
#define DPS_DISABLESPI
#ifndef DPS_DISABLESPI
#include <SPI.h>
#endif
// #include <Wire.h>

typedef struct
{
    int8_t (*bus_read)(uint8_t, uint8_t,uint8_t *, uint8_t);
	int8_t (*bus_write)(uint8_t, uint8_t,uint8_t *, uint8_t);
} i2c_bus;

// 从util/DpsRegister.h整合的结构体定义
typedef struct
{
    uint8_t regAddress;
    uint8_t mask;
    uint8_t shift;
} RegMask_t;

typedef struct
{
    uint8_t regAddress;
    uint8_t length;
} RegBlock_t;

// 从util/dps_config.h整合的常量定义
///////////     DPS3xx    ///////////
#define DPS3xx__PROD_ID 0x00
#define DPS3xx__SPI_WRITE_CMD 0x00U
#define DPS3xx__SPI_READ_CMD 0x80U
#define DPS3xx__SPI_RW_MASK 0x80U
#define DPS3xx__SPI_MAX_FREQ 1000000U

#define DPS3xx__OSR_SE 3U

// DPS3xx has 10 milliseconds of spare time for each synchronous measurement / per second for asynchronous measurements
// this is for error prevention on friday-afternoon-products :D
// you can set it to 0 if you dare, but there is no warranty that it will still work
#define DPS3xx__BUSYTIME_FAILSAFE 10U
#define DPS3xx__MAX_BUSYTIME ((1000U - DPS3xx__BUSYTIME_FAILSAFE) * DPS__BUSYTIME_SCALING)

#define DPS3xx__REG_ADR_SPI3W 0x09U
#define DPS3xx__REG_CONTENT_SPI3W 0x01U

///////////     common    ///////////

// slave address same for 3xx
#define DPS__FIFO_SIZE 32
#define DPS__STD_SLAVE_ADDRESS 0x77U
#define DPS__RESULT_BLOCK_LENGTH 3
#define NUM_OF_COMMON_REGMASKS 16

#define DPS__MEASUREMENT_RATE_1 0
#define DPS__MEASUREMENT_RATE_2 1
#define DPS__MEASUREMENT_RATE_4 2
#define DPS__MEASUREMENT_RATE_8 3
#define DPS__MEASUREMENT_RATE_16 4
#define DPS__MEASUREMENT_RATE_32 5
#define DPS__MEASUREMENT_RATE_64 6
#define DPS__MEASUREMENT_RATE_128 7

#define DPS__OVERSAMPLING_RATE_1 DPS__MEASUREMENT_RATE_1
#define DPS__OVERSAMPLING_RATE_2 DPS__MEASUREMENT_RATE_2
#define DPS__OVERSAMPLING_RATE_4 DPS__MEASUREMENT_RATE_4
#define DPS__OVERSAMPLING_RATE_8 DPS__MEASUREMENT_RATE_8
#define DPS__OVERSAMPLING_RATE_16 DPS__MEASUREMENT_RATE_16
#define DPS__OVERSAMPLING_RATE_32 DPS__MEASUREMENT_RATE_32
#define DPS__OVERSAMPLING_RATE_64 DPS__MEASUREMENT_RATE_64
#define DPS__OVERSAMPLING_RATE_128 DPS__MEASUREMENT_RATE_128

// we use 0.1 ms units for time calculations, so 10 units are one millisecond
#define DPS__BUSYTIME_SCALING 10U

#define DPS__NUM_OF_SCAL_FACTS 8

// status code
#define DPS__SUCCEEDED 0
#define DPS__FAIL_UNKNOWN -1
#define DPS__FAIL_INIT_FAILED -2
#define DPS__FAIL_TOOBUSY -3
#define DPS__FAIL_UNFINISHED -4

// 从util/dps3xx_config.h整合的定义
#define DPS3xx_NUM_OF_REGMASKS 16

enum Interrupt_source_3xx_e
{
    DPS3xx_NO_INTR = 0,
    DPS3xx_PRS_INTR = 1,
    DPS3xx_TEMP_INTR = 2,
    DPS3xx_BOTH_INTR = 3,
    DPS3xx_FIFO_FULL_INTR = 4,
};

namespace dps
{

    /**
     * @brief Operating mode.
     *
     */
    enum Mode
    {
        IDLE = 0x00,
        CMD_PRS = 0x01,
        CMD_TEMP = 0x02,
        CMD_BOTH = 0x03, // only for DPS422
        CONT_PRS = 0x05,
        CONT_TMP = 0x06,
        CONT_BOTH = 0x07
    };

    enum RegisterBlocks_e
    {
        PRS = 0, // pressure value
        TEMP,    // temperature value
    };

    const RegBlock_t registerBlocks[2] = {
        {0x00, 3},
        {0x03, 3},
    };

    /**
     * @brief registers for configuration and flags; these are the same for both 3xx and 422, might need to be adapted for future sensors
     *
     */
    enum Config_Registers_e
    {
        TEMP_MR = 0, // temperature measure rate
        TEMP_OSR,    // temperature measurement resolution
        PRS_MR,      // pressure measure rate
        PRS_OSR,     // pressure measurement resolution
        MSR_CTRL,    // measurement control
        FIFO_EN,

        TEMP_RDY,
        PRS_RDY,
        INT_FLAG_FIFO,
        INT_FLAG_TEMP,
        INT_FLAG_PRS,
    };

    const RegMask_t config_registers[NUM_OF_COMMON_REGMASKS] = {
        {0x07, 0x70, 4}, // TEMP_MR
        {0x07, 0x07, 0}, // TEMP_OSR
        {0x06, 0x70, 4}, // PRS_MR
        {0x06, 0x07, 0}, // PRS_OSR
        {0x08, 0x07, 0}, // MSR_CTRL
        {0x09, 0x02, 1}, // FIFO_EN

        {0x08, 0x20, 5}, // TEMP_RDY
        {0x08, 0x10, 4}, // PRS_RDY
        {0x0A, 0x04, 2}, // INT_FLAG_FIFO
        {0x0A, 0x02, 1}, // INT_FLAG_TEMP
        {0x0A, 0x01, 0}, // INT_FLAG_PRS
    };

} // namespace dps

namespace dps3xx
{

    enum Registers_e
    {
        PROD_ID = 0,
        REV_ID,
        TEMP_SENSOR,    // internal vs external
        TEMP_SENSORREC, // temperature sensor recommendation
        TEMP_SE,        // temperature shift enable (if temp_osr>3)
        PRS_SE,         // pressure shift enable (if prs_osr>3)
        FIFO_FL,        // FIFO flush
        FIFO_EMPTY,     // FIFO empty
        FIFO_FULL,      // FIFO full
        INT_HL,
        INT_SEL, // interrupt select
    };

    const RegMask_t registers[DPS3xx_NUM_OF_REGMASKS] = {
        {0x0D, 0x0F, 0}, // PROD_ID
        {0x0D, 0xF0, 4}, // REV_ID
        {0x07, 0x80, 7}, // TEMP_SENSOR
        {0x28, 0x80, 7}, // TEMP_SENSORREC
        {0x09, 0x08, 3}, // TEMP_SE
        {0x09, 0x04, 2}, // PRS_SE
        {0x0C, 0x80, 7}, // FIFO_FL
        {0x0B, 0x01, 0}, // FIFO_EMPTY
        {0x0B, 0x02, 1}, // FIFO_FULL
        {0x09, 0x80, 7}, // INT_HL
        {0x09, 0x70, 4}, // INT_SEL
    };

    const RegBlock_t coeffBlock = {0x10, 18};
} // namespace dps3xx

#ifdef __cplusplus
extern "C" {
#endif

class DpsClass
{
public:
    // constructor
    DpsClass(void);
    // destructor
    ~DpsClass(void);
    i2c_bus i2c_bus_ops;
    void (*delay_msec)(uint16_t);/**< delay function pointer*/	
    /**
     * I2C begin function with standard address
     */
    // void begin(rt_device_t &bus);

    /**
     * Standard I2C begin function
     *
     * @param &bus:             I2CBus which connects MC to the sensor
     * @param slaveAddress:     I2C address of the sensor (0x77 or 0x76)
     */
    void begin(uint8_t slaveAddress);

#ifndef DPS_DISABLESPI
    /**
     * SPI begin function for Dps3xx with 4-wire SPI
     */
    void begin(SPIClass &bus, int32_t chipSelect);
#endif

#ifndef DPS_DISABLESPI
    /**
     * Standard SPI begin function
     *
     * @param &bus:             SPI bus which connects MC to Dps3xx
     * @param chipSelect:       Number of the CS line for the Dps3xx
     * @param threeWire:        1 if Dps3xx is connected with 3-wire SPI
     *                          0 if Dps3xx is connected with 4-wire SPI (standard)
     */
    void begin(SPIClass &bus, int32_t chipSelect, uint8_t threeWire);
#endif

    /**
     * End function for Dps3xx
     * Sets the sensor to idle mode
     */
    void end(void);

    /**
     * returns the Product ID of the connected Dps3xx sensor
     */
    uint8_t getProductId(void);

    /**
     * returns the Revision ID of the connected Dps3xx sensor
     */
    uint8_t getRevisionId(void);

    /**
     * Sets the Dps3xx to standby mode
     *
     * @return  status code
     */
    int16_t standby(void);

    /**
     * performs one temperature measurement
     *
     * @param &result:      reference to a float value where the result will be written
     * @return 	status code
     */
    int16_t measureTempOnce(float &result);

    /**
     * performs one temperature measurement with specified oversamplingRate
     *
     * @param &result:              reference to a float where the result will be written
     * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
     *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128,
     *                              which are defined as integers 0 - 7
     *                              The number of measurements equals to 2^n, if the value written to 
     *                              the register field is n. 2^n internal measurements are combined to
     *                              return a more exact measurement
     * @return   status code
     */
    int16_t measureTempOnce(float &result, uint8_t oversamplingRate);

    /**
     * starts a single temperature measurement
     *
     * @return 	status code
     */
    int16_t startMeasureTempOnce(void);

    /**
     * starts a single temperature measurement with specified oversamplingRate
     *
     * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
     *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128, which are defined as integers 0 - 7
     * @return  status code
     */
    int16_t startMeasureTempOnce(uint8_t oversamplingRate);

    /**
     * performs one pressure measurement
     *
     * @param &result:              reference to a float value where the result will be written
     * @return 	status code
     */
    int16_t measurePressureOnce(float &result);

    /**
     * performs one pressure measurement with specified oversamplingRate
     *
     * @param &result:              reference to a float where the result will be written
     * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
     *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
     * @return  status code
     */
    int16_t measurePressureOnce(float &result, uint8_t oversamplingRate);

    /**
     * starts a single pressure measurement
     *
     * @return 	status code
     */
    int16_t startMeasurePressureOnce(void);

    /**
     * starts a single pressure measurement with specified oversamplingRate
     *
     * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2,
     *                              DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
     * @return  status code
     */
    int16_t startMeasurePressureOnce(uint8_t oversamplingRate);

    /**
     * gets the result a single temperature or pressure measurement in °C or Pa
     *
     * @param &result:              reference to a float value where the result will be written
     * @return 	status code
     */
    int16_t getSingleResult(float &result);

    /**
     * starts a continuous temperature measurement with specified measurement rate and oversampling rate
     * If measure rate is n and oversampling rate is m, the DPS3xx performs 2^(n+m) internal measurements per second.
     * The DPS3xx cannot operate with high precision and high speed at the same time. Consult the datasheet for more information.
     *
     * @param measureRate:          DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
     * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
     *
     * @return  status code
     *
     */
    int16_t startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate);

    /**
     * starts a continuous temperature measurement with specified measurement rate and oversampling rate
     *
     * @param measureRate:          DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
     * @param oversamplingRate:     DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
     * @return  status code
     */
    int16_t startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate);

    /**
     * starts a continuous temperature and pressure measurement with specified measurement rate and oversampling rate for temperature and pressure measurement respectively.
     *
     * @param tempMr:               measure rate for temperature
     * @param tempOsr:              oversampling rate for temperature
     * @param prsMr:                measure rate for pressure
     * @param prsOsr:               oversampling rate for pressure
     * @return  status code
     */
    int16_t startMeasureBothCont(uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr);

    /**
     * Gets the interrupt status flag of the FIFO
     *
     * @return 	    1 if the FIFO is full and caused an interrupt
     *              0 if the FIFO is not full or FIFO interrupt is disabled
     *              -1 on fail
     */
    int16_t getIntStatusFifoFull(void);

    /**
     * Gets the interrupt status flag that indicates a finished temperature measurement
     *
     * @return 	    1 if a finished temperature measurement caused an interrupt;
     *              0 if there is no finished temperature measurement or interrupts are disabled;
     *              -1 on fail.
     */
    int16_t getIntStatusTempReady(void);

    /**
     * Gets the interrupt status flag that indicates a finished pressure measurement
     *
     * @return      1 if a finished pressure measurement caused an interrupt;
     *              0 if there is no finished pressure measurement or interrupts are disabled;
     *              -1 on fail.
     */
    int16_t getIntStatusPrsReady(void);

    /**
     * Function to fix a hardware problem on some devices
     * You have this problem if you measure a temperature which is too high (e.g. 60°C when temperature is around 20°C)
     * Call correctTemp() directly after begin() to fix this issue
     */
    int16_t correctTemp(void);

protected:
    // scaling factor table
    static const int32_t scaling_facts[DPS__NUM_OF_SCAL_FACTS];

    dps::Mode m_opMode;

    // flags
    uint8_t m_initFail;

    uint8_t m_productID;
    uint8_t m_revisionID;

    // settings
    uint8_t m_tempMr;
    uint8_t m_tempOsr;
    uint8_t m_prsMr;
    uint8_t m_prsOsr;

    // compensation coefficients for both dps3xx and dps422
    int32_t m_c00;
    int32_t m_c10;
    int32_t m_c01;
    int32_t m_c11;
    int32_t m_c20;
    int32_t m_c21;
    int32_t m_c30;

    // last measured scaled temperature (necessary for pressure compensation)
    float m_lastTempScal;

    // bus specific
    uint8_t m_SpiI2c; // 0=SPI, 1=I2C

    // used for I2C
    // rt_device_t m_i2cbus;
    uint8_t m_slaveAddress;

#ifndef DPS_DISABLESPI
    // used for SPI
    SPIClass *m_spibus;
    int32_t m_chipSelect;
    uint8_t m_threeWire;
#endif
    /**
     * Initializes the sensor.
     * This function has to be called from begin()
     * and requires a valid bus initialization.
     */
    virtual void init(void) = 0;

    /**
     * reads the compensation coefficients from the sensor
     * this is called once from init(), which is called from begin()
     *
     * @return 	0 on success, -1 on fail
     */
    virtual int16_t readcoeffs(void) = 0;

    /**
     * Sets the Operation Mode of the sensor
     *
     * @param opMode:           the new OpMode as defined by dps::Mode; CMD_BOTH should not be used for DPS3xx
     * @return                  0 on success,
     *                          -1 on fail
     */
    int16_t setOpMode(uint8_t opMode);

    /**
     * Configures temperature measurement
     *
     * @param temp_mr:          DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
     * @param temp_osr:         DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
     *
     * @return 	0 normally or -1 on fail
     */
    virtual int16_t configTemp(uint8_t temp_mr, uint8_t temp_osr);

    /**
     * Configures pressure measurement
     *
     * @param prs_mr:           DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
     * @param prs_osr:          DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
     * @return 	                0 normally or
     *                          -1 on fail
     */
    virtual int16_t configPressure(uint8_t prs_mr, uint8_t prs_osr);

    virtual int16_t flushFIFO() = 0;

    virtual float calcTemp(int32_t raw) = 0;

    virtual float calcPressure(int32_t raw) = 0;

    int16_t enableFIFO();

    int16_t disableFIFO();

    /**
     * calculates the time that the sensor needs for 2^mr measurements with an oversampling rate of 2^osr (see table "pressure measurement time (ms) versus oversampling rate")
     * Note that the total measurement time for temperature and pressure must not be more than 1 second.
     * Timing behavior of pressure and temperature sensors can be considered the same.
     *
     * @param mr:               DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
     * @param osr:              DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
     * @return time that the sensor needs for this measurement
     */
    uint16_t calcBusyTime(uint16_t temp_rate, uint16_t temp_osr);

    /**
     * reads the next raw value from the FIFO
     *
     * @param value:  the raw pressure or temperature value read from the pressure register blocks, where the LSB of PRS_B0 marks whether the value is a temperature or a pressure.
     *
     * @return          -1 on fail
     *                  0 if result is a temperature raw value
     *                  1 if result is a pressure raw value
     */
    int16_t getFIFOvalue(int32_t *value);

    /**
     * Gets the results from continuous measurements and writes them to given arrays
     *
     * @param *tempBuffer:      The start address of the buffer where the temperature results are written
     *                          If this is NULL, no temperature results will be written out
     * @param &tempCount:       The size of the buffer for temperature results.
     *                          When the function ends, it will contain the number of bytes written to the buffer.
     * @param *prsBuffer:       The start address of the buffer where the pressure results are written
     *                          If this is NULL, no pressure results will be written out
     * @param &prsCount:        The size of the buffer for pressure results.
     *                          When the function ends, it will contain the number of bytes written to the buffer.
     * @param reg               The FIFO empty register field; needed since this field is different for each sensor
     * @return  status code
     */
    int16_t getContResults(float *tempBuffer, uint8_t &tempCount, float *prsBuffer, uint8_t &prsCount, RegMask_t reg);

    /**
     * reads a byte from the sensor
     *
     * @param regAddress:        Address that has to be read
     * @return  register content or -1 on fail
     */
    int16_t readByte(uint8_t regAddress);

#ifndef DPS_DISABLESPI
    /**
     * reads a byte from the sensor via SPI
     * this function is automatically called by readByte
     * if the sensor is connected via SPI
     *
     * @param regAddress:       Address that has to be read
     * @return  register content or -1 on fail
     */
    int16_t readByteSPI(uint8_t regAddress);
#endif
    /**
     * reads a block from the sensor
     *
     * @param regAddress:       Address that has to be read
     * @param length:           Length of data block
     * @param buffer:           Buffer where data will be stored
     * @return  number of bytes that have been read successfully, which might not always equal to length due to rx-Buffer overflow etc.
     */
    int16_t readBlock(RegBlock_t regBlock, uint8_t *buffer);

#ifndef DPS_DISABLESPI
    /**
     * reads a block from the sensor via SPI
     *
     * @param regAddress:       Address that has to be read
     * @param length:           Length of data block
     * @param readBuffer:       Buffer where data will be stored
     * @return  number of bytes that have been read successfully, which might not always equal to length due to rx-Buffer overflow etc.
     */
    int16_t readBlockSPI(RegBlock_t regBlock, uint8_t *readBuffer);
#endif
    /**
     * writes a byte to a given register of the sensor without checking
     *
     * @param regAddress:       Address of the register that has to be updated
     * @param data:             Byte that will be written to the register
     * @return  0 if byte was written successfully or
     *          -1 on fail
     */
    int16_t writeByte(uint8_t regAddress, uint8_t data);

    /**
     * writes a byte to a register of the sensor
     *
     * @param regAddress:       Address of the register that has to be updated
     * @param data:             Byte that will be written to the register
     * @param check:            If this is true, register content will be read after writing
     *                          to check if update was successful
     * @return  0 if byte was written successfully or
     *          -1 on fail
     */
    int16_t writeByte(uint8_t regAddress, uint8_t data, uint8_t check);

#ifndef DPS_DISABLESPI
    /**
     * writes a byte to a register of the sensor via SPI
     *
     * @param regAddress:       Address of the register that has to be updated
     * @param data:             Byte that will be written to the register
     * @param check:            If this is true, register content will be read after writing
     *                          to check if update was successful
     * @return  0 if byte was written successfully or
     *          -1 on fail
     */
    int16_t writeByteSpi(uint8_t regAddress, uint8_t data, uint8_t check);
#endif

    /**
     * updates a bit field of the sensor without checking
     *
     * @param regMask:          Mask of the register that has to be updated
     * @param data:             BitValues that will be written to the register
     * @return  0 if byte was written successfully or
     *          -1 on fail
     */
    int16_t writeByteBitfield(uint8_t data, RegMask_t regMask);

    /**
     * updates a bit field of the sensor
     *
     * regMask:             Mask of the register that has to be updated
     * data:                BitValues that will be written to the register
     * check:               enables/disables check after writing; 0 disables check.
     *                      if check fails, -1 will be returned
     * @return  0 if byte was written successfully or
     *          -1 on fail
     */
    int16_t writeByteBitfield(uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift, uint8_t check);

    /**
     * reads a bit field from the sensor
     * regMask:             Mask of the register that has to be updated
     * data:                BitValues that will be written to the register
     * @return  read and processed bits or -1 on fail
     */
    int16_t readByteBitfield(RegMask_t regMask);

    /**
     * @brief converts non-32-bit negative numbers to 32-bit negative numbers with 2's complement
     *
     * @param raw The raw number of less than 32 bits
     * @param length The bit length
     */
    void getTwosComplement(int32_t *raw, uint8_t length);

    /**
     * @brief Get a raw result from a given register block
     *
     * @param raw The address where the raw value is to be written
     * @param reg The register block to be read from
     * @return status code
     */
    int16_t getRawResult(int32_t *raw, RegBlock_t reg);
};

class Dps3xx : public DpsClass
{
public:
    static Dps3xx& getInstance()
    {
        static Dps3xx instance;
        return instance;
    }
    
    int16_t getContResults(float *tempBuffer, uint8_t &tempCount, float *prsBuffer, uint8_t &prsCount);

    /**
     * @brief Set the source of interrupt (FIFO full, measurement values ready)
     *
     * @param intr_source Interrupt source as defined by Interrupt_source_3xx_e
     * @param polarity
     * @return status code
     */
    int16_t setInterruptSources(uint8_t intr_source, uint8_t polarity = 1);

// protected:
    uint8_t m_tempSensor;

    // compensation coefficients
    int32_t m_c0Half;
    int32_t m_c1;

    /////// implement pure virtual functions ///////
    void init(void) override;
    int16_t configTemp(uint8_t temp_mr, uint8_t temp_osr) override;
    int16_t configPressure(uint8_t prs_mr, uint8_t prs_osr) override;
    int16_t readcoeffs(void) override;
    int16_t flushFIFO() override;
    float calcTemp(int32_t raw) override;
    float calcPressure(int32_t raw) override;
};

#ifdef __cplusplus
}
#endif

#endif
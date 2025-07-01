/**
 * @file GoertekBaro.hpp
 * @brief 气压计头文件
 * @details 歌尔气压力传感器驱动
 * @author R&D Embedded Departments
 * @email gaofeng@zerozero.cn
 * @version 1.0.0
 * @date 2024-06-014
 * @license Copyright (c) 2014-2019 Beijing Zero Zero Infinity \
                                Technology Co., Ltd all right reserved
 */

#pragma once
#include <rtthread.h>

#include "stdint.h"

typedef bool (*i2cReaddataMult)(uint8_t, uint8_t *, uint8_t);
typedef bool (*i2cReaddata)(uint8_t, uint8_t *);
typedef bool (*i2cWritedata)(uint8_t, uint8_t);
typedef void (*timeDelayMs)(uint16_t);

#pragma pack(push, 4)
typedef struct
{
    uint32_t id;
    uint64_t timestamp_ms;
    float temperature_celsius;
    float pressure_pa;
} BaroData;
#pragma pack(pop)

enum class GoertekBaroRegister : uint8_t
{
    PSR_B2_ADDR = 0x00,
    PSR_B1_ADDR = 0x01,
    PSR_B0_ADDR = 0x02,
    TMP_B2_ADDR = 0x03,
    TMP_B1_ADDR = 0x04,
    TMP_B0_ADDR = 0x05,
    PRS_CFG_ADDR = 0x06,
    TMP_CFG_ADDR = 0x07,
    MEAS_CFG_ADDR = 0x08,
    CFG_REG_ADDR = 0x09,
    INT_STS_ADDR = 0x0A,
    FIFO_STS_ADDR = 0x0B,
    RESET_ADDR = 0x0C,
    PRODUCT_ID_ADDR = 0x0D,
    COEF_ADDR = 0x10,
};

class GoertekBaro
{
   public:
    enum ScalingCofficients
    {
        OSR_SF_1 = 524288,
        OSR_SF_2 = 1572864,
        OSR_SF_4 = 3670016,
        OSR_SF_8 = 7864320,
        OSR_SF_16 = 253952,
        OSR_SF_32 = 516096,
        OSR_SF_64 = 1040384,
        OSR_SF_128 = 2088960,
    };

    // struct to hold calibration coefficients read from device
    struct CaliCoefficientReg
    {
        int32_t c0;   // 12bit
        int16_t c1;   // 12bit
        int32_t c00;  // 20bit
        int32_t c10;  // 20bit
        int16_t c01;  // 16bit
        int16_t c11;  // 16bit
        int16_t c20;  // 16bit
        int16_t c21;  // 16bit
        int16_t c30;  // 16bit
        int16_t c31;  // 12bit
        int16_t c40;  // 12bit
    };

    // enum for setting/getting device operating mode
    enum OperationMode
    {
        GoertekBaro_MODE_IDLE = 0b00000000,
        GoertekBaro_MODE_COMMAND_PRESSURE = 0b00000001,
        GoertekBaro_MODE_COMMAND_TEMPERATURE = 0b00000010,
        GoertekBaro_MDOE_BACKGROUND_PRESSURE = 0b00000101,
        GoertekBaro_MODE_BACKGROUND_TEMPERATURE = 0b00000110,
        GoertekBaro_MODE_BACKGROUND_ALL = 0b00000111,
    };

    // enum of measurement rates for pressure
    enum PressureMeasureRate
    {
        PM_RATE_1_HZ = 0b00000000,
        PM_RATE_2_HZ = 0b00010000,
        PM_RATE_4_HZ = 0b00100000,
        PM_RATE_8_HZ = 0b00110000,
        PM_RATE_16_HZ = 0b01000000,
        PM_RATE_32_HZ = 0b01010000,
        PM_RATE_64_HZ = 0b01100000,
        PM_RATE_128_HZ = 0b01110000,
    };

    // enum of measurement rates for temperature
    enum TemperatureMeasureRate
    {
        TMP_RATE_1_HZ = 0b00000000,
        TMP_RATE_2_HZ = 0b00010000,
        TMP_RATE_4_HZ = 0b00100000,
        TMP_RATE_8_HZ = 0b00110000,
        TMP_RATE_16_HZ = 0b01000000,
        TMP_RATE_32_HZ = 0b01010000,
        TMP_RATE_64_HZ = 0b01100000,
        TMP_RATE_128_HZ = 0b01110000,
    };

    // enum of oversampling rates for pressure and temperature
    enum OverSampleRate
    {
        OSR_1_TIME = 0b00000000,
        OSR_2_TIMES = 0b00000001,
        OSR_4_TIMES = 0b00000010,
        OSR_8_TIMES = 0b00000011,
        OSR_16_TIMES = 0b00000100,
        OSR_32_TIMES = 0b00000101,
        OSR_64_TIMES = 0b00000110,
        OSR_128_TIMES = 0b00000111,
    };

    // enum of temperature source
    enum TemperatureSource
    {
        TMP_EXT_ASIC = 0x00,
        TMP_EXT_MEMS = 0x80,
    };

    enum Product_ID
    {
        SPL06_001,
        SPA06_003
    };
    GoertekBaro();
    GoertekBaro(int id);
    ~GoertekBaro();
    bool Init();
    int DebugInit();
    bool Deinit();
    bool Read(BaroData &data);
    bool ReadWhoAmI(uint8_t *who_am_i);
    void ExecutePeriodically();
    int32_t GetTwosComplement(int32_t raw, uint8_t length);
    bool Resume();
    bool Standby();

   public:
    static constexpr uint8_t WHO_AM_I_001 = 0x10;
    static constexpr uint8_t WHO_AM_I_003 = 0x11;
    static constexpr uint8_t GoertekBaro_I2C_SLAVE_ADDRESS = 0x76;

    uint8_t baro_model_;

    void funcRegisterI2c(i2cReaddata read_func, i2cWritedata write_func, i2cReaddataMult read_mult_func);
    void funRegisterTimer(timeDelayMs delay_ms_func);

   protected:
    i2cReaddata i2c_read_func;
    i2cWritedata i2c_write_func;
    i2cReaddataMult i2c_read_mult_func;
    timeDelayMs delay_msec;
    int id_;
    BaroData data_;
    bool has_inited_;

   private:
    // LittleEndian BufferToInt32_;
    ScalingCofficients tmperature_osr_scale_coeff_;  // Temperature scaling coefficient
    ScalingCofficients prs_osr_scale_coeff_;         // Pressure scaling coefficient
    CaliCoefficientReg calib_coeffs_;                // Calibration coefficients index
    OperationMode dev_mode_;                         // Current operating mode of device
    PressureMeasureRate pressure_mr_;                // Current measurement readout rate (ODR)
    TemperatureMeasureRate temperature_mr_;          // Current measurement readout rate (ODR)
    OverSampleRate temperature_osr_;                 // Current oversampling rate(OSR) for temperature
    OverSampleRate pressure_osr_;                    // Current oversampling rate(OSR) for puressure
    TemperatureSource tmperature_ext_;               // Temperature ACIS or MEMS

    bool ReadCalibCoefficient();
    bool ConfigParam();
    ScalingCofficients GetScalingCoef(OverSampleRate osr);
};

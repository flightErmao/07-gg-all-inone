#pragma once
#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

// 错误码定义
#define MPU_EOK 0     // 无错误
#define MPU_ERROR -1  // 一般错误

// C接口函数声明
#ifdef __cplusplus
extern "C" {
#endif

extern int drv_mpu6500_init();
extern int drv_mpu6500_set_delay(void (*delay_ms)(unsigned int));
extern int drv_mpu6500_set_i2c_funcs(int8_t (*write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                                     int8_t (*read_func)(uint8_t, uint8_t, uint8_t*, uint8_t));
extern int drv_mpu6500_read(int pos, void* data, int size);
extern int drv_mpu6500_get_accel(int16_t* ax, int16_t* ay, int16_t* az);
extern int drv_mpu6500_get_gyro(int16_t* gx, int16_t* gy, int16_t* gz);
extern int drv_mpu6500_get_temp(float* temp);
extern int drv_mpu6500_self_test();
extern int drv_mpu6500_set_i2c_addr(uint8_t addr);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * @class mpu6500
 * @brief mpu6500 惯性测量单元驱动类
 */
class mpu6500 {
 public:
  /**
   * @brief 设备结构体
   */
  typedef struct {
    void* user_data;  // 用户数据指针，可存储I2C设备句柄
  } mpu_device_t;

  /**
   * @brief 延时函数指针类型
   */
  typedef void (*delay_ms_func_t)(unsigned int ms);

  /**
   * @brief 获取单例实例
   */
  static mpu6500& instance();

  /**
   * @brief 构造函数
   */
  explicit mpu6500();

  /**
   * @brief 析构函数
   */
  ~mpu6500();

  /**
   * @brief 初始化设备
   * @return 错误码
   */
  int init();

  /**
   * @brief 设置I2C地址
   * @param addr 地址值
   */
  void setI2cAddr(uint8_t addr);

  /**
   * @brief 设置延时函数
   * @param func 延时函数指针
   */
  void setDelayMs(delay_ms_func_t func);

  /**
   * @brief 设置I2C写函数
   * @param func I2C写函数指针
   * @return 错误码
   */
  int8_t setI2cBusWrite(int8_t (*func)(uint8_t, uint8_t, uint8_t*, uint8_t));

  /**
   * @brief 设置I2C读函数
   * @param func I2C读函数指针
   * @return 错误码
   */
  int8_t setI2cBusRead(int8_t (*func)(uint8_t, uint8_t, uint8_t*, uint8_t));

  /**
   * @brief 读取传感器数据
   * @param pos 寄存器地址
   * @param data 数据缓冲区
   * @param size 读取字节数
   * @return 读取的字节数或错误码
   */
  int read_data(int pos, void* data, int size);

  /**
   * @brief 获取加速度计数据
   * @param ax X轴加速度
   * @param ay Y轴加速度
   * @param az Z轴加速度
   * @return 错误码
   */
  uint8_t getAccelerometer(int16_t* ax, int16_t* ay, int16_t* az);

  /**
   * @brief 获取陀螺仪数据
   * @param gx X轴角速度
   * @param gy Y轴角速度
   * @param gz Z轴角速度
   * @return 错误码
   */
  uint8_t getGyroscope(int16_t* gx, int16_t* gy, int16_t* gz);

  /**
   * @brief 获取温度值
   * @param temp 温度值
   * @return 错误码
   */
  uint8_t getTemperature(float* temp);

  /**
   * @brief 自检函数
   * @return 0: 成功, 1: 失败
   */
  uint8_t selfTest();

 private:
  uint8_t dev_addr_ = {};
  bool is_init_ = false;
  mpu_device_t device_ = {};  // 补充与cpp一致的成员
  uint8_t buffer[14];

  int8_t (*i2c_bus_write_)(uint8_t, uint8_t, uint8_t*, uint8_t) = nullptr;
  int8_t (*i2c_bus_read_)(uint8_t, uint8_t, uint8_t*, uint8_t) = nullptr;
  delay_ms_func_t delay_ms_ = nullptr;  // 延时函数指针

  // I2C底层操作函数
  int8_t i2cBusRead(uint8_t devAddress, uint8_t memAddress, uint8_t len, uint8_t* data);
  int8_t i2cBusWrite(uint8_t devAddress, uint8_t memAddress, uint8_t len, uint8_t* data);
  int8_t i2cBusReadByte(uint8_t devAddress, uint8_t memAddress, uint8_t* data);
  int8_t i2cBusWriteByte(uint8_t devAddress, uint8_t memAddress, uint8_t data);

  // 位操作函数
  int8_t i2cdevReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data);
  int8_t i2cdevReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
  int8_t i2cdevWriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
  int8_t i2cdevWriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

  // 内部延时函数
  void delayMs(unsigned int ms);

  // 初始化相关函数
  void reset();                                // 复位MPU6500
  uint8_t getDeviceID();                       // 获取设备ID
  void setSleepEnabled(bool enabled);          // 设置睡眠模式
  void setClockSource(uint8_t source);         // 设置时钟源
  void setTempSensorEnabled(bool enabled);     // 设置温度传感器
  void setIntEnabled(bool enabled);            // 设置中断
  void setI2CBypassEnabled(bool enabled);      // 设置I2C旁路模式
  void setFullScaleGyroRange(uint8_t range);   // 设置陀螺量程
  void setFullScaleAccelRange(uint8_t range);  // 设置加速度计量程
  void setAccelDLPF(uint8_t bandwidth);        // 设置加速度计低通滤波
  void setDLPFMode(uint8_t mode);              // 设置数字低通滤波模式
  uint8_t setRate(uint16_t rate);

  // 下面为在 cpp 中新增的内部方法声明
  int mpu6500Init();
  void mpu6500SetWaitForExternalSensorEnabled(bool enabled);
  void mpu6500SetInterruptMode(bool mode);
  void mpu6500SetInterruptDrive(bool drive);
  void mpu6500SetInterruptLatch(bool latch);
  void mpu6500SetInterruptLatchClear(bool clear);
  void mpu6500SetSlaveReadWriteTransitionEnabled(bool enabled);
  void mpu6500SetMasterClockSpeed(uint8_t speed);
  void mpu6500SetI2CMasterModeEnabled(bool enabled);
  void mpu6500SetIntDataReadyEnabled(bool enabled);
  int mpu6500SlaveSensorInit();
};
#endif

/*!
 * @file INA219.h
 * @copyright   Copyright (c) 2023 MARUM, Deep-Sea Engineering
 * @License     MIT
 * @author Pablo Gutiérrez F.
 * @date  2023-06-20
 */
 
#ifndef _INA219_H
#define _INA219_H

#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <stdlib.h>
#include <cstdint>

#include <iostream>
#include <time.h>
#include <stdint.h>
#include <string.h>

#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <memory>

/** default I2C address **/
#define INA219_I2C_ADDRESS1 (0x40)///< I2C ADDRESS 1
#define INA219_I2C_ADDRESS2 (0x41)///< I2C ADDRESS 2
#define INA219_I2C_ADDRESS3 (0x44)///< I2C ADDRESS 3
#define INA219_I2C_ADDRESS4 (0x45)///< I2C ADDRESS 4

/** read **/
#define INA219_READ (0x01)

/** config register address **/
#define INA219_REG_CONFIG (0x00)

/** reset bit **/
#define INA219_CONFIG_RESET (0x8000) // Reset Bit

/** mask for bus voltage range **/
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask

/** bus voltage range values **/
enum {
  INA219_CONFIG_BVOLTAGERANGE_16V = (0x0000), // 0-16V Range
  INA219_CONFIG_BVOLTAGERANGE_32V = (0x2000), // 0-32V Range
};

/** mask for gain bits **/
#define INA219_CONFIG_GAIN_MASK (0x1800) // Gain Mask

/** values for gain bits **/
enum {
  INA219_CONFIG_GAIN_1_40MV = (0x0000),  // Gain 1, 40mV Range
  INA219_CONFIG_GAIN_2_80MV = (0x0800),  // Gain 2, 80mV Range
  INA219_CONFIG_GAIN_4_160MV = (0x1000), // Gain 4, 160mV Range
  INA219_CONFIG_GAIN_8_320MV = (0x1800), // Gain 8, 320mV Range
};

/** mask for bus ADC resolution bits **/
#define INA219_CONFIG_BADCRES_MASK (0x0780)

/** values for bus ADC resolution **/
enum {
  INA219_CONFIG_BADCRES_9BIT = (0x0000),  // 9-bit bus res = 0..511
  INA219_CONFIG_BADCRES_10BIT = (0x0080), // 10-bit bus res = 0..1023
  INA219_CONFIG_BADCRES_11BIT = (0x0100), // 11-bit bus res = 0..2047
  INA219_CONFIG_BADCRES_12BIT = (0x0180), // 12-bit bus res = 0..4097
  INA219_CONFIG_BADCRES_12BIT_2S_1060US = (0x0480), // 2 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_4S_2130US = (0x0500), // 4 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_8S_4260US = (0x0580), // 8 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_16S_8510US = (0x0600), // 16 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_32S_17MS = (0x0680), // 32 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_64S_34MS = (0x0700), // 64 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_128S_69MS = (0x0780), // 128 x 12-bit bus samples averaged together
};

/** mask for shunt ADC resolution bits **/
#define INA219_CONFIG_SADCRES_MASK  (0x0078) // Shunt ADC Resolution and Averaging Mask

/** values for shunt ADC resolution **/
enum {
  INA219_CONFIG_SADCRES_9BIT_1S_84US = (0x0000),   // 1 x 9-bit shunt sample
  INA219_CONFIG_SADCRES_10BIT_1S_148US = (0x0008), // 1 x 10-bit shunt sample
  INA219_CONFIG_SADCRES_11BIT_1S_276US = (0x0010), // 1 x 11-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_1S_532US = (0x0018), // 1 x 12-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_2S_1060US = (0x0048), // 2 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_4S_2130US = (0x0050), // 4 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_8S_4260US = (0x0058), // 8 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_16S_8510US = (0x0060), // 16 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_32S_17MS = (0x0068), // 32 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_64S_34MS = (0x0070), // 64 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_128S_69MS = (0x0078), // 128 x 12-bit shunt samples averaged together
};

/** mask for operating mode bits **/
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask

/** values for operating mode **/
enum {
  INA219_CONFIG_MODE_POWERDOWN = 0x00,       /**< power down */
  INA219_CONFIG_MODE_SVOLT_TRIGGERED = 0x01, /**< shunt voltage triggered */
  INA219_CONFIG_MODE_BVOLT_TRIGGERED = 0x02, /**< bus voltage triggered */
  INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = 0x03,                         /**< shunt and bus voltage triggered */
  INA219_CONFIG_MODE_ADCOFF = 0x04, /**< ADC off */
  INA219_CONFIG_MODE_SVOLT_CONTINUOUS = 0x05, /**< shunt voltage continuous */
  INA219_CONFIG_MODE_BVOLT_CONTINUOUS = 0x06, /**< bus voltage continuous */
  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x07, /**< shunt and bus voltage continuous */
};

/** shunt voltage register **/
#define INA219_REG_SHUNTVOLTAGE (0x01)

/** bus voltage register **/
#define INA219_REG_BUSVOLTAGE (0x02)

/** power register **/
#define INA219_REG_POWER (0x03)

/** current register **/
#define INA219_REG_CURRENT (0x04)

/** calibration register **/
#define INA219_REG_CALIBRATION (0x05)


class INA219
{

public:

    INA219(){};

    void setCalibration_32V_2A();
    void setCalibration_32V_1A();
    void setCalibration_16V_400mA();

    void setShuntSizeInOhms(float shuntSize);
    void setShuntVoltOffset_mV(float offs);
	
    void reset();

    /**
     * @fn getBusVoltage_V
     * @brief  get the BusVoltage （Voltage of IN- to GND)
     * @return Voltage unit:V
     */
    float getBusVoltage_V();

    /**
     * @fn getShuntVoltage_mV
     * @brief  get the ShuntVoltage （Voltage of the sampling resistor, IN+ to NI-)
     * @return Voltage unit:mV
     */
    float getShuntVoltage_mV();
    
    /**
     * @fn getCurrent_mA
     * @brief get the Current(Current flows across IN+ and IN-.
     * @n If the current flows from IN+ to IN-, the reading is positive. 
     * @n If the current flows from IN- to IN+, the reading is negative)
     * @return Current unit:mA
     */
    float getCurrent_mA();
    
    /**
     * @fn getPower_mW
     * @brief Get power
     * @details the power resolution read directly from the module is 20mW 
     * @return power unit：mW
     */
     float getPower_mW();


protected:

    uint32_t ina219_calValue;
    uint32_t ina219_currentDivider_mA;
    float ina219_powerMultiplier_mW;
    float shuntFactor = 1.0;
    float shuntVoltageOffset = 0.0;
    bool offsetIsSet = false;

    virtual bool scan() = 0;
    int16_t readInaReg(uint8_t reg);
    void writeInaReg(uint8_t reg, uint16_t value);
    virtual bool writeLen(uint8_t reg, const uint8_t *buffer, uint8_t len) = 0;
    virtual bool readLen(uint8_t reg, uint8_t *buffer, uint8_t len) = 0;
    
};

class INA219_IIC : public INA219
{
public:

    INA219_IIC(int file, uint8_t i2caddr);
    bool scan();

protected:
    bool writeLen(uint8_t reg, const uint8_t *buffer, uint8_t len);
    bool readLen(uint8_t reg, uint8_t *buffer, uint8_t len);

    uint8_t   _addr;
    int _file;
};

#endif

/*!
 * @file INA219.cpp
 * @copyright   Copyright (c) 2023 MARUM, Deep-Sea Engineering
 * @License     MIT
 * @author Pablo Gutiérrez F.
 * @date  2023-06-20
 */

#include "INA219.h"

INA219_IIC::INA219_IIC(int file, uint8_t i2caddr): INA219()
{
   _file = file; _addr = i2caddr;
}

void INA219::setShuntSizeInOhms(float shuntSize){
    shuntFactor = shuntSize / 0.1;
}

void INA219::setShuntVoltOffset_mV(float offs){
    shuntVoltageOffset = offs;
    offsetIsSet = true; 
}

float INA219::getBusVoltage_V()
{
    return (float) (readInaReg(INA219_REG_BUSVOLTAGE) >> 3) * 0.004;
}

float INA219::getShuntVoltage_mV()
{
    return (float)(readInaReg(INA219_REG_SHUNTVOLTAGE) * 0.01 - shuntVoltageOffset);
}

float INA219::getCurrent_mA()
{
    float offsetCurrent = 0.0;
    writeInaReg(INA219_REG_CALIBRATION, ina219_calValue);
    int16_t valDec = readInaReg(INA219_REG_CURRENT);
    if(offsetIsSet){
        offsetCurrent = static_cast<int16_t>(shuntVoltageOffset * 100.0 * ina219_calValue / 4096.0);
    }
    return (float)((valDec - offsetCurrent) / (ina219_currentDivider_mA * shuntFactor));
}

float INA219::getPower_mW()
{
    writeInaReg(INA219_REG_CALIBRATION, ina219_calValue);
    float valueDec = (float)readInaReg(INA219_REG_POWER);
    valueDec *= ina219_powerMultiplier_mW / shuntFactor;
    return valueDec;
}

void INA219::setCalibration_32V_2A() {

  ina219_calValue = 4096;
  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 10.0; // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2.0; // Power LSB = 1mW per bit (2/1)
  

  // Set Calibration register to 'Cal' calculated above
  writeInaReg(INA219_REG_CALIBRATION, ina219_calValue);
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  writeInaReg(INA219_REG_CONFIG, config);
}

void INA219::setCalibration_32V_1A() {

  ina219_calValue = 10240;
  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
  ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

  // Set Calibration register to 'Cal' calculated above
  writeInaReg(INA219_REG_CALIBRATION, ina219_calValue);
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  writeInaReg(INA219_REG_CONFIG, config);
}

void INA219::setCalibration_16V_400mA() {

  ina219_calValue = 8192;
  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 20.0;    // Current LSB = 50uA per bit (1000/50 = 20)
  ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  writeInaReg(INA219_REG_CALIBRATION, ina219_calValue);
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  writeInaReg(INA219_REG_CONFIG, config);
}

void INA219::reset()
{
    writeInaReg(INA219_REG_CONFIG, INA219_CONFIG_RESET);
}


int16_t INA219::readInaReg(uint8_t reg)
{
    uint8_t buf[2];
    readLen(reg, buf, sizeof(buf));
    return (int16_t)(buf[0] <<8) | buf[1];
}

void INA219::writeInaReg(uint8_t reg, uint16_t value)
{
    //uint8_t buffer[2] = {static_cast<uint8_t>(value & 0xff), 
    //                     static_cast<uint8_t>(value >> 8)};

    uint8_t buffer[2];
    buffer[1] = value & 0xFF;
    buffer[0] = (value >>= 8) & 0xFF;
    writeLen(reg, buffer, sizeof(buffer));
}


/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/
/**************************************************************************/
bool INA219_IIC::readLen(uint8_t reg, uint8_t * buffer, uint8_t len)
{
  uint8_t buff[len];   
  memset(buff,'\0', len);
  
  buff[0] = reg;
  write(_file,buff,1);
  
  if (read(_file, buff, len) != len) {
    return false; 
  }
  else{
    for(int i=0; i<len; i++){
      buffer[i] = buff[i];
    }
    return true;
  }
}

/**************************************************************************/
/*!
    @brief  Write the specified number of bytes over I2C
*/
/**************************************************************************/
bool INA219_IIC::writeLen(uint8_t reg, const uint8_t *buffer, uint8_t len)
{
  uint8_t length = len + 1; 
  uint8_t buff[length];

  memset(buff,'\0', length);

  buff[0] = reg;
  for(int i=1; i < length; i++)
    buff[i] = buffer[i-1];

  if(write(_file,buff,length) != length){
        return false;
  }else{
        return true;
  }
}

bool INA219_IIC::scan()
{
    if (ioctl(_file, I2C_SLAVE, _addr) < 0) 
        return false;
    else return true;
}

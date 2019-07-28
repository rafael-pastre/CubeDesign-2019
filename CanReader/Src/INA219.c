#include "INA219.h"

void wireWriteRegister (uint8_t reg, uint16_t value)
{
	HAL_StatusTypeDef status = 0;

	uint8_t buffer[3];
	buffer[0] = reg; // register
	buffer[1] = (value >> 8); // MSB
	buffer[2] = value & 0xff; // LSB
	status = HAL_I2C_Master_Transmit(&ina219_hi2c1, ina219_i2caddr<<1, buffer, 3, 100);
}

void wireReadRegister(uint8_t reg, uint16_t *value)
{
	HAL_StatusTypeDef status = 0;

	status = HAL_I2C_Master_Transmit(&ina219_hi2c1, ina219_i2caddr<<1, &reg, 1, 100);
	HAL_Delay(1); // Max 12-bit conversion time is 586us per sample

	uint8_t buffer[2] = {0,0};
	status = HAL_I2C_Master_Receive(&ina219_hi2c1, ina219_i2caddr<<1, buffer, 2, 100);
	*value = (buffer[0]<<8) | buffer[1];
}

void setCalibration_32V_2A()
{
  ina219_calValue = 4096;

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2;     // Power LSB = 1mW per bit (2/1)

  // Set Calibration register to 'Cal' calculated above
  wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wireWriteRegister(INA219_REG_CONFIG, config);
}

void setCalibration_32V_1A()
{
  ina219_calValue = 10240;

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
  ina219_powerMultiplier_mW = 1;         // Power LSB = 800mW per bit

  // Set Calibration register to 'Cal' calculated above
  wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wireWriteRegister(INA219_REG_CONFIG, config);
}

void setCalibration_16V_400mA() {

  ina219_calValue = 8192;

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
  ina219_powerMultiplier_mW = 1;     // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wireWriteRegister(INA219_REG_CONFIG, config);
}

void ina219_init(I2C_HandleTypeDef hi2c1, uint8_t ina219_addr) {
  ina219_i2caddr = ina219_i2caddr;
  ina219_currentDivider_mA = 0;
  ina219_powerMultiplier_mW = 0;
  ina219_hi2c1 = hi2c1;

  // Set chip to large range config values to start
  setCalibration_32V_2A();
}

int16_t getBusVoltage_raw() {
  uint16_t value;
  wireReadRegister(INA219_REG_BUSVOLTAGE, &value);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)((value >> 3) * 4);
}

int16_t getShuntVoltage_raw() {
  uint16_t value;
  wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}

int16_t getCurrent_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

  // Now we can safely read the CURRENT register!
  wireReadRegister(INA219_REG_CURRENT, &value);

  return (int16_t)value;
}

int16_t getPower_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

  // Now we can safely read the POWER register!
  wireReadRegister(INA219_REG_POWER, &value);

  return (int16_t)value;
}

float getShuntVoltage_mV() {
  int16_t value;
  value = getShuntVoltage_raw();
  return value * 0.01;
}

float getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
  return value * 0.001;
}

float getCurrent_mA() {
  float valueDec = getCurrent_raw();
  valueDec /= ina219_currentDivider_mA;
  return valueDec;
}

float getPower_mW() {
  float valueDec = getPower_raw();
  valueDec *= ina219_powerMultiplier_mW;
  return valueDec;
}




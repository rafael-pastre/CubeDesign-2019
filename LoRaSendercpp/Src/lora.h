/*
 * lora.h
 *
 *  Created on: Jul 1, 2019
 *      Author: Rafael Pastre
 */

#ifndef LORA_H_
#define LORA_H_

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

//#include <Arduino.h>
//#include <spi.h>


#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class LoRaClass {
public:
  LoRaClass();

  int begin(long frequency);
  void end();

  int beginPacket(int implicitHeader = false);
  void delayUS(unsigned int us);
  int endPacket(bool async = false);

  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();
  long packetFrequencyError();

  // from Print
  virtual size_t write(uint8_t byte);
  size_t write(const char *str) {
    if (str == NULL) return 0;
    return write((const uint8_t *)str, strlen(str));
  }
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int));

  void receive(int size = 0);

  void idle();
  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();
  void enableInvertIQ();
  void disableInvertIQ();

  void setOCP(uint8_t mA); // Over Current Protection control

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  uint8_t random();

  //void setPins(GPIO_TypeDef * nss_gpio_port, uint16_t nss_pin, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  void setPins(GPIO_TypeDef * nss_gpio_port, uint16_t nss_pin, GPIO_TypeDef * reset_gpio_port, uint16_t reset_pin, IRQn_Type dio0_IRQ);
  void setSPI(SPI_HandleTypeDef* hspi);
  void setTIM(TIM_HandleTypeDef* htim);

  void dumpRegisters();

  void onDio0Rise();

  /**
   * print methods
   */
  size_t print(const char[]);
  size_t print(char);
  size_t print(unsigned char, int = DEC);
  size_t print(int, int = DEC);
  size_t print(unsigned int, int = DEC);
  size_t print(long, int = DEC);
  size_t print(unsigned long, int = DEC);
  size_t print(double, int = 2);
//private:
  size_t printNumber(unsigned long, uint8_t);
  size_t printFloat(double, uint8_t);

//private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();
  bool isTransmitting();

  int getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

//private:
  //SPISettings _spiSettings;
  //SPIClass* _spi;
  SPI_HandleTypeDef* _hspi;
  TIM_HandleTypeDef* _htim;
  GPIO_TypeDef * _nss_gpio_port;
  uint16_t _nss_pin;
  GPIO_TypeDef * _reset_gpio_port;
  uint16_t _reset_pin;
  IRQn_Type _dio0_IRQ;
  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
};

#endif /* LORA_H_ */

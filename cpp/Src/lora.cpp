/*
 * lora.cpp
 *
 *  Created on: Jul 1, 2019
 *      Author: Rafael Pastre
 */

// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "lora.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

LoRaClass::LoRaClass() :
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL)
{
  // overide Stream timeout value
  //setTimeout(0);
}

int LoRaClass::begin(long frequency)
{

  if (_reset_pin != -1) {

    // perform reset
	HAL_GPIO_WritePin(_reset_gpio_port, _reset_pin, GPIO_PIN_RESET);	//digitalWrite(_reset, LOW);
    HAL_Delay(10);
    HAL_GPIO_WritePin(_reset_gpio_port, _reset_pin, GPIO_PIN_SET);		//digitalWrite(_reset, HIGH);
    HAL_Delay(10);
  }

  // start SPI
  //_spi->begin();

  // check version
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    return 0;
  }

  // put in sleep mode
  sleep();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  setTxPower(17);

  // put in standby mode
  idle();

  return 1;
}

//sleep(), SPI
void LoRaClass::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  //_spi->end();
}

//LATER
int LoRaClass::beginPacket(int implicitHeader)
{
  if (isTransmitting()) {
    return 0;
  }

  // put in standby mode
  idle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

//NEW
void LoRaClass::delayUS(unsigned int us){
	_htim->Instance->CNT = 0;
	while(_htim->Instance->CNT < us);
	return;
}

//writeRegister(), delayMicroseconds(), yield()
int LoRaClass::endPacket(bool async)
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (async) {
    // grace time is required for the radio
    delayUS(150);
  } else {
    // wait for TX done
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      //yield();
    }
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 1;
}

//readRegister(), writeRegister()
bool LoRaClass::isTransmitting()
{
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }

  if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

//LATER
int LoRaClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

//readRegister()
int LoRaClass::packetRssi()
{
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

//readRegister()
float LoRaClass::packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

//readRegister(), getSignalBandwidth()
long LoRaClass::packetFrequencyError()
{
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return static_cast<long>(fError);
}

//readRegister(), writeRegister()
size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

//readRegister(), writeRegister()
size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

//readRegister()
int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

//readRegister(), available()
int LoRaClass::read()
{
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

//writeRegister(), readRegister()
int LoRaClass::peek()
{
  if (!available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

//DONE
void LoRaClass::flush()
{
}

//LATER
void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {

    writeRegister(REG_DIO_MAPPING_1, 0x00);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    //attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
    HAL_NVIC_EnableIRQ(_dio0_IRQ);
  } else {
    //detachInterrupt(digitalPinToInterrupt(_dio0));
    HAL_NVIC_DisableIRQ(_dio0_IRQ);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

//writeRegister(), explicitHeaderMode()
void LoRaClass::receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

//writeRegister()
void LoRaClass::idle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

//writeRegister()
void LoRaClass::sleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

//writeRegister()
void LoRaClass::setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      writeRegister(REG_PA_DAC, 0x87);
      setOCP(140);
    } else {
      if (level < 2) {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      setOCP(100);
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

//writeRegister()
void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

//readRegister()
int LoRaClass::getSpreadingFactor()
{
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

//writeRegister(), setLdoFlag(), readRegister()
void LoRaClass::setSpreadingFactor(int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

//readRegister()
long LoRaClass::getSignalBandwidth()
{
  uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

//writeRegister(), setLdoFlag()
void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  setLdoFlag();
}

//writeRegister(), readRegister(), getSignalBandwidth(), getSpreadingFactor(), bitWrite()
void LoRaClass::setLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);

  //bitWrite(config3, 3, ldoOn);
  //NEW
  if(ldoOn == true){
	  config3 ^= 0b00001000;
  }
  else{
	  config3 &= 0b11110111;
  }

  writeRegister(REG_MODEM_CONFIG_3, config3);
}

//readRegister()
void LoRaClass::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

//writeRegister()
void LoRaClass::setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

//writeRegister()
void LoRaClass::setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

//writeRegister()
void LoRaClass::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

//writeRegister()
void LoRaClass::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

//writeRegister()
void LoRaClass::enableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x66);
  writeRegister(REG_INVERTIQ2, 0x19);
}

//writeRegister()
void LoRaClass::disableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x27);
  writeRegister(REG_INVERTIQ2, 0x1d);
}

//writeRegister()
void LoRaClass::setOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

//readRegister()
uint8_t LoRaClass::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

//internal setting
void LoRaClass::setPins(GPIO_TypeDef * nss_gpio_port, uint16_t nss_pin, GPIO_TypeDef * reset_gpio_port, uint16_t reset_pin, IRQn_Type dio0_IRQ)
{
	_nss_gpio_port = nss_gpio_port;
	_nss_pin = nss_pin,
	_reset_gpio_port = reset_gpio_port;
	_reset_pin = reset_pin;
	_dio0_IRQ = dio0_IRQ;
}

//internal setting
void LoRaClass::setSPI(SPI_HandleTypeDef* hspi)
{
  _hspi = hspi;
}

//NEW
void LoRaClass::setTIM(TIM_HandleTypeDef* htim)
{
  _htim = htim;
}

//Stream, readRegister()
//NEW
void LoRaClass::dumpRegisters()
{
  for (int i = 0; i < 128; i++) {
    printf("0x");
    printf("%x",i);
    printf(": 0x");
    printf("%x\n",readRegister(i));
  }
}

//writeRegister()
void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

//writeRegister()
void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

//readRegister(), writeRegister(), _onReceive
void LoRaClass::handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    if (_onReceive) {
      _onReceive(packetLength);
    }

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}

//singleTransfer()
uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

//singleTransfer()
void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

//NEW
uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;

  HAL_GPIO_WritePin(_nss_gpio_port, _nss_pin, GPIO_PIN_RESET);	//digitalWrite(_ss, LOW);

  //_spi->beginTransaction(_spiSettings);
  //_spi->transfer(address);
  //response = _spi->transfer(value);
  //_spi->endTransaction();

  HAL_SPI_Transmit (_hspi, &address, sizeof(uint8_t), 1000);
  HAL_SPI_TransmitReceive(_hspi, &value, &response, 2*sizeof(uint8_t), 1000);

  HAL_GPIO_WritePin(_nss_gpio_port, _nss_pin, GPIO_PIN_SET);	//digitalWrite(_ss, HIGH);

  return response;
}

//handleDio0Rise()
void LoRaClass::onDio0Rise()
{
  LoRa.handleDio0Rise();
}

LoRaClass LoRa;

#pragma once
#include "Arduino.h"
#if CONFIG_IDF_TARGET_ESP32
#include "ArduinoESP32DMADesc.h"
#include "esp32-hal-spi.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "driver/spi_master.h"
#include <SPI.h>

struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};
#define SPI_LL_DMA_FIFO_RST_MASK (SPI_AHBM_RST | SPI_AHBM_FIFO_RST)
class DMASPI : public SPIClass {
public:
  DMASPI(uint8_t host);
  ~DMASPI();
  void begin(int sckPin, int misoPin, int mosiPin, int csPin);
  void initDMA(uint32_t txDescs, uint32_t rxDescs, uint16_t dataLength);
  void startDMA(bool continuous = false);
  void startDMA(DMADesc *tx,DMADesc *rx, bool continuous = false);
  void stopDMA();
  void setHardwareCSEnabled(bool enabled);
  inline uint32_t getCurrentRXDesc(){ return spi->dev->dma_inlink_dscr; }
  inline uint32_t getCurrentRXBuffer(){ return spi->dev->dma_inlink_dscr_bf1; }
  inline uint32_t getNextRXDesc(){ return spi->dev->dma_inlink_dscr_bf0; }
  inline uint32_t getCurrentTXDesc(){ return spi->dev->dma_outlink_dscr; }
  inline uint32_t getCurrentTXBuffer(){ return spi->dev->dma_outlink_dscr_bf1; }
  inline uint32_t getNextTXDesc(){ return spi->dev->dma_outlink_dscr_bf0; }
  inline void triggerTransfer(){
    /*Serial.print("RX Desc:");
    Serial.println(getCurrentRXDesc());
    Serial.print("RX Buffer:");
    Serial.println(getCurrentRXBuffer());
    Serial.print("RX Next Desc:");
    Serial.println(getNextRXDesc());
    Serial.print("TX Desc:");
    Serial.println(getCurrentTXDesc());
    Serial.print("TX Buffer:");
    Serial.println(getCurrentTXBuffer());
    Serial.print("TX Next Desc:");
    Serial.println(getNextTXDesc());*/

    spi->dev->dma_in_link.start = 1;    //使能一次DMA操作
    spi->dev->dma_out_link.start = 1;   //使能一次DMA操作
    spi->dev->cmd.usr = 1;              //使能一次SPI操作
  }
  DMADesc *dmaDescTX;
  DMADesc *dmaDescRX;
  uint32_t txDescCount;
  uint32_t rxDescCount;
private:
  uint16_t dmaDataLength;
  spi_t *spi;
  uint8_t SPIHost;
};

#endif
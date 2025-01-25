#pragma once
#include "Arduino.h"
#if CONFIG_IDF_TARGET_ESP32
#include "ArduinoESP32DMADesc.h"
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
  inline void triggerTransfer(){
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
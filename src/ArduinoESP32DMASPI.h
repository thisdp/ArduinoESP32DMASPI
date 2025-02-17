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
  uint32_t readSPIBuffer() { return spi->dev->data_buf[0]; }
  void setHardwareCSEnabled(bool enabled);
  inline uint32_t getCurrentRXDesc(){ return spi->dev->dma_inlink_dscr; }
  inline uint32_t getCurrentRXBuffer(){ return spi->dev->dma_inlink_dscr_bf1; }
  inline uint32_t getNextRXDesc(){ return spi->dev->dma_inlink_dscr_bf0; }
  inline uint32_t getCurrentTXDesc(){ return spi->dev->dma_outlink_dscr; }
  inline uint32_t getCurrentTXBuffer(){ return spi->dev->dma_outlink_dscr_bf1; }
  inline uint32_t getNextTXDesc(){ return spi->dev->dma_outlink_dscr_bf0; }
  inline void triggerDMARX(){
    dma_rx_reset(); //复位指针，取消DMA缓存
    dma_rx_start(); //使能一次DMA操作，但需要等待
  }
  inline void triggerDMATX(){
    dma_tx_reset(); //复位指针，取消DMA缓存
    dma_tx_start(); //使能一次DMA操作，但需要等待
  }
  inline void triggerSPI(){
    spi->dev->cmd.usr = 1;              //使能一次SPI操作
  }
  inline void triggerTransfer(){
    dma_rx_start(); //使能一次DMA操作
    dma_tx_start(); //使能一次DMA操作
    triggerSPI();
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
  }
  inline void manualTransfer(uint8_t *data, uint32_t length){
    uint32_t alignedLength = ((length + 4 - 1) / 4) * 4;
    uint8_t *memBuffer = new uint8_t[alignedLength];
    memset(memBuffer,0,alignedLength);
    memcpy(memBuffer,data,length);
    for(uint32_t i = 0; i < alignedLength/4; i++){
      spi->dev->data_buf[i] = ((uint32_t *)memBuffer)[i];
    }
    delete memBuffer;
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

  //ll driver
  inline void dma_tx_reset(){
    spi->dev->dma_conf.out_rst = 1;
    spi->dev->dma_conf.out_rst = 0;
  }
  inline void dma_rx_reset(){
    spi->dev->dma_conf.in_rst = 1;
    spi->dev->dma_conf.in_rst = 0;
  }
  inline void dma_rx_prepare(){
    spi->dev->dma_conf.val |= SPI_LL_DMA_FIFO_RST_MASK;
    spi->dev->dma_conf.val &= ~SPI_LL_DMA_FIFO_RST_MASK;
  }
  inline void dma_tx_prepare(){
    spi->dev->dma_conf.val |= SPI_LL_DMA_FIFO_RST_MASK;
    spi->dev->dma_conf.val &= ~SPI_LL_DMA_FIFO_RST_MASK;
  }
  inline void dma_tx_load(DMADesc *desc){ spi->dev->dma_out_link.addr = (int)desc& 0xFFFFF; }
  inline void dma_tx_start(){ spi->dev->dma_out_link.start = 1; }
  inline void dma_tx_stop(){ spi->dev->dma_out_link.stop = 0; }
  inline void dma_rx_load(DMADesc *desc){ spi->dev->dma_in_link.addr = (int)desc& 0xFFFFF; }
  inline void dma_rx_start(){ spi->dev->dma_in_link.start = 1; }
  inline void dma_rx_stop(){ spi->dev->dma_in_link.start = 0; }
};

#endif
#pragma once
#include "Arduino.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "ArduinoESP32DMADesc.h"
#include "driver/spi_master.h"
#include <SPI.h>
#include <esp_private/gdma.h>


struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};

class GDMASPI : public SPIClass {
public:
  typedef bool(*GDMASPICallBack)(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data);
  GDMASPI(uint8_t host);
  ~GDMASPI();
  void begin(int sckPin, int misoPin, int mosiPin, int csPin);
  void registerCallBack(GDMASPICallBack cb);
  void initDMA(uint32_t txDescs, uint32_t rxDescs, uint16_t dataLength);
  void startDMA(DMADesc *tx,DMADesc *rx);
  void startDMA();
  void stopDMA();
  void setHardwareCSEnabled(bool enabled);
  inline spi_t *bus(){ return spi; }
  inline void triggerTransfer(){
    spi->dev->cmd.usr = 1;
  }
  gdma_channel_handle_t dmaChannelRX;
  gdma_channel_handle_t dmaChannelTX;
  DMADesc *dmaDescTX;
  DMADesc *dmaDescRX;
  uint32_t txDescCount;
  uint32_t rxDescCount;
protected:
  uint16_t dmaDataLength;
  spi_t *spi;
  uint8_t SPIHost;
};
#endif
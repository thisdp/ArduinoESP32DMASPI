#ifndef ESP32SPIDMA_H
#define ESP32SPIDMA_H

#include "Arduino.h"
#include "driver/spi_master.h"
#include <SPI.h>
class DMADesc {
public:
  void begin(uint8_t *inputBuffer, uint16_t bufferSize);
  inline bool hasError(){ return err_eof; }
  inline bool isLast(){ return suc_eof; }
  inline void setLast(bool isLast){ suc_eof = isLast; }
  inline bool isOwnedByDMA(){ return owner; }
  inline void setOwnedByDMA(bool ownedByDMA){ owner = ownedByDMA; }
  inline void linkTo(DMADesc* nextDMADesc){ next = nextDMADesc; }
  inline void linkTo(DMADesc& nextDMADesc){ next = &nextDMADesc; }
  inline void *getBuffer() { return buffer; };
private:
  struct {
      uint32_t size : 12;         /*!< Buffer size */
      uint32_t length : 12;       /*!< Number of valid bytes in the buffer */
      uint32_t reversed24_27 : 4; /*!< Reserved */
      uint32_t err_eof : 1;       /*!< Whether the received buffer contains error */
      uint32_t reserved29 : 1;    /*!< Reserved */
      uint32_t suc_eof : 1;       /*!< Whether the descriptor is the last one in the link */
      uint32_t owner : 1;         /*!< Who is allowed to access the buffer that this descriptor points to */
  };                          /*!< Descriptor Word 0 */
  void *buffer;                   /*!< Pointer to the buffer */
  DMADesc *next;  /*!< Pointer to the next descriptor (set to NULL if the descriptor is the last one, e.g. suc_eof=1) */
};

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
  //typedef bool(*DMASPICallBack)(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data);
  DMASPI(uint8_t host);
  void begin(int sckPin, int misoPin, int mosiPin, int csPin);
  //void registerCallBack(DMASPICallBack cb);
  void initDMA(uint32_t descs, uint16_t length);
  void startDMA(bool continuous = false);
  void stopDMA();
  inline void triggerTransfer(){
    spi->dev->dma_in_link.start = 1;    //使能一次DMA操作
    spi->dev->dma_out_link.start = 1;   //使能一次DMA操作
    spi->dev->cmd.usr = 1;              //使能一次SPI操作
  }
  DMADesc *dmaDescTX;
  DMADesc *dmaDescRX;
  uint8_t *dmaBufferTX;
  uint8_t *dmaBufferRX;
private:
  spi_t *spi;
  uint8_t SPIHost;
};

#endif // ESP32SPIDMA_H
#ifndef ESP32SPIDMA_H
#define ESP32SPIDMA_H

#include "Arduino.h"
#include "driver/spi_master.h"
#include <SPI.h>
#include <esp_private/gdma.h>
class GDMADesc {
public:
  void end();
  void begin(uint16_t bufferSize);
  inline bool hasError(){ return err_eof; }
  inline bool isLast(){ return suc_eof; }
  inline void setLast(bool isLast){ suc_eof = isLast; }
  inline bool isOwnedByDMA(){ return owner; }
  inline void setOwnedByDMA(bool ownedByDMA){ owner = ownedByDMA; }
  inline void linkNext(GDMADesc* nextGDMADesc){ next = nextGDMADesc; }
  inline void linkNext(GDMADesc& nextGDMADesc){ next = &nextGDMADesc; }
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
  GDMADesc *next;  /*!< Pointer to the next descriptor (set to NULL if the descriptor is the last one, e.g. suc_eof=1) */
};

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
  void startDMA(GDMADesc *tx,GDMADesc *rx);
  void startDMA();
  void stopDMA();
  void setHardwareCSEnabled(bool enabled);
  inline spi_t *bus(){ return spi; }
  inline void triggerTransfer(){
    spi->dev->cmd.usr = 1;
  }
  gdma_channel_handle_t dmaChannelRX;
  gdma_channel_handle_t dmaChannelTX;
  GDMADesc *dmaDescTX;
  GDMADesc *dmaDescRX;
  uint32_t txDescCount;
  uint32_t rxDescCount;
protected:
  uint16_t dmaDataLength;
  spi_t *spi;
  uint8_t SPIHost;
};

#endif // ESP32SPIDMA_H
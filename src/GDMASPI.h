#ifndef ESP32SPIDMA_H
#define ESP32SPIDMA_H

#include "Arduino.h"
#include "driver/spi_master.h"
#include "soc/gdma_struct.h"
#include "hal/gdma_hal.h"
#include <SPI.h>
#include <esp_private/gdma.h>
#include "soc/soc_caps.h"

#include <stdlib.h>
#include <sys/cdefs.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "soc/periph_defs.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/periph_ctrl.h"
#include "esp_private/gdma.h"
#include "esp_heap_caps.h"
#include "hal/gdma_hal.h"
#include "hal/gdma_ll.h"
#include "soc/gdma_periph.h"
#include "soc/soc_memory_types.h"

static const char *TAG = "gdma";

#if CONFIG_GDMA_ISR_IRAM_SAFE || CONFIG_GDMA_CTRL_FUNC_IN_IRAM
#define GDMA_MEM_ALLOC_CAPS    (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#else
#define GDMA_MEM_ALLOC_CAPS    MALLOC_CAP_DEFAULT
#endif

#if CONFIG_GDMA_ISR_IRAM_SAFE
#define GDMA_INTR_ALLOC_FLAGS  (ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED)
#else
#define GDMA_INTR_ALLOC_FLAGS  ESP_INTR_FLAG_INTRDISABLED
#endif

#define GDMA_INVALID_PERIPH_TRIG  (0x3F)
#define SEARCH_REQUEST_RX_CHANNEL (1 << 0)
#define SEARCH_REQUEST_TX_CHANNEL (1 << 1)

typedef struct gdma_platform_t gdma_platform_t;
typedef struct gdma_group_t gdma_group_t;
typedef struct gdma_pair_t gdma_pair_t;
typedef struct gdma_channel_t gdma_channel_t;
typedef struct gdma_tx_channel_t gdma_tx_channel_t;
typedef struct gdma_rx_channel_t gdma_rx_channel_t;

/**
 * GDMA driver consists of there object class, namely: Group, Pair and Channel.
 * Channel is allocated when user calls `gdma_new_channel`, its lifecycle is maintained by user.
 * Pair and Group are all lazy allocated, their life cycles are maintained by this driver.
 * We use reference count to track their life cycles, i.e. the driver will free their memory only when their reference count reached to 0.
 *
 * We don't use an all-in-one spin lock in this driver, instead, we created different spin locks at different level.
 * For platform, it has a spinlock, which is used to protect the group handle slots and reference count of each group.
 * For group, it has a spinlock, which is used to protect group level stuffs, e.g. hal object, pair handle slots and reference count of each pair.
 * For pair, it has a spinlock, which is used to protect pair level stuffs, e.g. channel handle slots, occupy code.
 */

struct gdma_platform_t {
    portMUX_TYPE spinlock;                 // platform level spinlock
    gdma_group_t *groups[SOC_GDMA_GROUPS]; // array of GDMA group instances
    int group_ref_counts[SOC_GDMA_GROUPS]; // reference count used to protect group install/uninstall
};

struct gdma_group_t {
    int group_id;           // Group ID, index from 0
    gdma_hal_context_t hal; // HAL instance is at group level
    portMUX_TYPE spinlock;  // group level spinlock
    uint32_t tx_periph_in_use_mask; // each bit indicates which peripheral (TX direction) has been occupied
    uint32_t rx_periph_in_use_mask; // each bit indicates which peripheral (RX direction) has been occupied
    gdma_pair_t *pairs[SOC_GDMA_PAIRS_PER_GROUP];  // handles of GDMA pairs
    int pair_ref_counts[SOC_GDMA_PAIRS_PER_GROUP]; // reference count used to protect pair install/uninstall
};

struct gdma_pair_t {
    gdma_group_t *group;        // which group the pair belongs to
    int pair_id;                // Pair ID, index from 0
    gdma_tx_channel_t *tx_chan; // pointer of tx channel in the pair
    gdma_rx_channel_t *rx_chan; // pointer of rx channel in the pair
    int occupy_code;            // each bit indicates which channel has been occupied (an occupied channel will be skipped during channel search)
    portMUX_TYPE spinlock;      // pair level spinlock
};

struct gdma_channel_t {
    gdma_pair_t *pair;  // which pair the channel belongs to
    intr_handle_t intr; // per-channel interrupt handle
    portMUX_TYPE spinlock;  // channel level spinlock
    gdma_channel_direction_t direction; // channel direction
    int periph_id; // Peripheral instance ID, indicates which peripheral is connected to this GDMA channel
    size_t sram_alignment;  // alignment for memory in SRAM
    size_t psram_alignment; // alignment for memory in PSRAM
    esp_err_t (*del)(gdma_channel_t *channel); // channel deletion function, it's polymorphic, see `gdma_del_tx_channel` or `gdma_del_rx_channel`
};

struct gdma_tx_channel_t {
    gdma_channel_t base; // GDMA channel, base class
    void *user_data;     // user registered DMA event data
    gdma_event_callback_t on_trans_eof; // TX EOF callback
};

struct gdma_rx_channel_t {
    gdma_channel_t base; // GDMA channel, base class
    void *user_data;     // user registered DMA event data
    gdma_event_callback_t on_recv_eof; // RX EOF callback
};

class GDMADesc {
public:
  void begin(uint8_t *inputBuffer, uint16_t bufferSize);
  inline bool hasError(){ return err_eof; }
  inline bool isLast(){ return suc_eof; }
  inline void setLast(bool isLast){ suc_eof = isLast; }
  inline bool isOwnedByDMA(){ return owner; }
  inline void setOwnedByDMA(bool ownedByDMA){ owner = ownedByDMA; }
  inline void linkTo(GDMADesc* nextGDMADesc){ next = nextGDMADesc; }
  inline void linkTo(GDMADesc& nextGDMADesc){ next = &nextGDMADesc; }
  inline void *getBuffer() { return buffer; };
  inline void print(){
    Serial.print(size);
    Serial.print("-");
    Serial.print(length);
    Serial.print("-");
    Serial.print(err_eof);
    Serial.print("-");
    Serial.print(suc_eof);
    Serial.print("-");
    Serial.print(owner);
    Serial.print("-");
    Serial.print((uint32_t)buffer);
    Serial.print("-");
    Serial.println((uint32_t)next);
  }
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
} __attribute__((aligned(4)));

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
  void begin(int sckPin, int misoPin, int mosiPin, int csPin);
  void registerCallBack(GDMASPICallBack cb);
  void initDMA(uint32_t descs, uint16_t length);
  void startDMA();
  void stopDMA();
  inline void triggerTransfer(){
    /*gdma_pair_t *pair = dmaChannelTX->pair;
    gdma_group_t *group = pair->group;
    gdma_hal_context_t *hal = &group->hal;
    gdma_dev_t *dev = hal->dev;
    int groupID = group->group_id;*/
    //Serial.println("----");
    //Serial.println(dev->channel[groupID].out.int_raw.val,BIN);
    //Serial.println(dev->channel[groupID].in.int_raw.val,BIN);
    /*for(uint8_t i=0;i<4;i++){
      Serial.print("TX Desc ");
      Serial.println(i);
      dmaDescTX[i].print();
    }
    for(uint8_t i=0;i<4;i++){
      Serial.print("RX Desc ");
      Serial.println(i);
      dmaDescRX[i].print();
    }*/
    
    /*for(uint8_t i=0;i<4;i++){
      dmaDescRX[i].setLast(false);
      dmaDescRX[i].setOwnedByDMA(true);
    }*/
    spi->dev->cmd.usr = 1;
  }
  gdma_channel_handle_t dmaChannelRX;
  gdma_channel_handle_t dmaChannelTX;
  GDMADesc *dmaDescTX;
  GDMADesc *dmaDescRX;
  uint8_t *dmaBufferTX;
  uint8_t *dmaBufferRX;
private:
  spi_t *spi;
  uint8_t SPIHost;
};

#endif // ESP32SPIDMA_H
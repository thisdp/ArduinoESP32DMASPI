#include "ArduinoESP32DMASPI.h"
#include "esp32-hal-spi.h"
#include "driver/periph_ctrl.h"
#include "esp32-hal-spi.h"
#include "esp32-hal.h"
#include "esp_attr.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "driver/periph_ctrl.h"
#include <driver/spi_common.h>

void DMADesc::begin(uint8_t *inputBuffer, uint16_t bufferSize){
  if(bufferSize >= 0xFFF) bufferSize = 0xFFF;
  buffer = inputBuffer;
  memset(buffer,0,bufferSize);
  size = bufferSize;
  length = bufferSize;
  owner = 1;
  suc_eof = 0;
  err_eof = 0;
}

DMASPI::DMASPI(uint8_t host):SPIHost(host){}

void DMASPI::begin(int sck, int miso, int mosi, int cs) {
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_DMA_CLK_EN);
  DPORT_SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, 3, SPIHost-1, ((SPIHost-1) * 2));
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);
  SPIClass::begin(sck, miso, mosi, cs);
  spi = bus();
  spi->dev->dma_conf.val |= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
  spi->dev->dma_in_link.start = 0;
  spi->dev->dma_out_link.start = 0;
  spi->dev->dma_in_link.addr = 0;
  spi->dev->dma_out_link.addr = 0;
  spi->dev->dma_conf.val &= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);
  spiAttachSS(spi, 0, cs);
  spiSSEnable(spi);
  dmaDescTX = 0;
  dmaDescRX = 0;
 

  spi->dev->mosi_dlen.usr_mosi_dbitlen  = 0;
  spi->dev->miso_dlen.usr_miso_dbitlen  = 0;
  
  SPISettings settings(10000000*8,SPI_MSBFIRST,SPI_MODE0);
  beginTransaction(settings);
  
}

void DMASPI::initDMA(uint32_t descs, uint16_t length){
  if(dmaDescTX) delete dmaDescTX;
  if(dmaDescRX) delete dmaDescRX;
  if(dmaBufferTX) delete dmaBufferTX;
  if(dmaBufferRX) delete dmaBufferRX;
  dmaDescTX = (DMADesc*)heap_caps_malloc(sizeof(DMADesc)*descs, MALLOC_CAP_DMA);
  dmaDescRX = (DMADesc*)heap_caps_malloc(sizeof(DMADesc)*descs, MALLOC_CAP_DMA);
  dmaBufferTX = (uint8_t*)heap_caps_malloc(length*descs, MALLOC_CAP_DMA);
  dmaBufferRX = (uint8_t*)heap_caps_malloc(length*descs, MALLOC_CAP_DMA);
  for(uint8_t i=0;i<descs;i++){
    dmaDescTX[i].begin(&(dmaBufferTX[i*length]),length);
    dmaDescTX[i].linkTo(dmaDescTX[(i+1)%descs]);
    dmaDescRX[i].begin(&(dmaBufferRX[i*length]),length);
    dmaDescRX[i].linkTo(dmaDescRX[(i+1)%descs]);
  }
  spi->dev->mosi_dlen.usr_mosi_dbitlen = length*8-1;
  spi->dev->miso_dlen.usr_miso_dbitlen = length*8-1;
}

void DMASPI::startDMA(bool continuous){
  spi->dev->dma_in_link.addr = (uint32_t)dmaDescRX & 0xFFFFF;
  spi->dev->dma_out_link.addr = (uint32_t)dmaDescTX & 0xFFFFF;
  spi->dev->dma_conf.dma_continue = continuous;
}

void DMASPI::stopDMA(){
  spi->dev->dma_conf.dma_continue = 0;
  spi->dev->dma_in_link.start = 0;
  spi->dev->dma_out_link.start = 0;
}
/*
void DMASPI::registerCallBack(DMASPICallBack cb){
  static gdma_rx_event_callbacks_t rx_cbs = {
    .on_recv_eof = cb
  };
  gdma_register_rx_event_callbacks(dmaChannelRX, &rx_cbs, NULL);  // Enable DMA transfer callback
}
*/
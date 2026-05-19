#include "ArduinoESP32DMASPI.h"
#if CONFIG_IDF_TARGET_ESP32
#include "driver/periph_ctrl.h"
#include "soc/dport_reg.h"
#include <driver/spi_common.h>

DMASPI::DMASPI(uint8_t host) : SPIClass(host),
  dmaDescTX(0),
  dmaDescRX(0),
  txDescCount(0),
  rxDescCount(0),
  dmaDataLength(0),
  spi(0),
  SPIHost(host){}

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
  if(dmaDescTX){
    for(uint32_t i=0;i<txDescCount;i++) dmaDescTX[i].end();
    heap_caps_free(dmaDescTX);
  }
  if(dmaDescRX){
    for(uint32_t i=0;i<rxDescCount;i++) dmaDescRX[i].end();
    heap_caps_free(dmaDescRX);
  }
  dmaDescTX = 0;
  dmaDescRX = 0;
  spi->dev->mosi_dlen.usr_mosi_dbitlen  = 0;
  spi->dev->miso_dlen.usr_miso_dbitlen  = 0;
  SPISettings settings(10000000*2,SPI_MSBFIRST,SPI_MODE0);
  beginTransaction(settings);
}

void DMASPI::setHardwareCSEnabled(bool enabled){
  if(enabled){
    spiSSEnable(spi);
  }else{
    spiSSDisable(spi);
  }
}

//如果dataLen不是8的倍数，则会出现错位问题，待解决
void DMASPI::initDMA(uint32_t txDescs, uint32_t rxDescs, uint16_t dataLen){
  if(dmaDescTX){
    for(uint32_t i=0;i<txDescCount;i++) dmaDescTX[i].end();
    heap_caps_free(dmaDescTX);
  }
  if(dmaDescRX){
    for(uint32_t i=0;i<rxDescCount;i++) dmaDescRX[i].end();
    heap_caps_free(dmaDescRX);
  }
  txDescCount = txDescs;
  rxDescCount = rxDescs;
  dmaDescTX = (DMADesc*)heap_caps_aligned_alloc(8,sizeof(DMADesc)*txDescs, MALLOC_CAP_DMA);
  dmaDescRX = (DMADesc*)heap_caps_aligned_alloc(8,sizeof(DMADesc)*rxDescs, MALLOC_CAP_DMA);
  for(uint32_t i=0;i<txDescs;i++) dmaDescTX[i].clear();
  for(uint32_t i=0;i<rxDescs;i++) dmaDescRX[i].clear();
  for(uint32_t i=0;i<txDescs;i++){
    dmaDescTX[i].begin(dataLen);
    dmaDescTX[i].linkNext(dmaDescTX[(i+1)%txDescs]);
  }
  for(uint32_t i=0;i<rxDescs;i++){
    dmaDescRX[i].begin(dataLen);
    dmaDescRX[i].linkNext(dmaDescRX[(i+1)%rxDescs]);
  }
  dmaDataLength = dataLen;
}

void DMASPI::startDMA(DMADesc *tx,DMADesc *rx, bool continuous){
  spi->dev->mosi_dlen.usr_mosi_dbitlen = dmaDataLength*8-1;
  spi->dev->miso_dlen.usr_miso_dbitlen = dmaDataLength*8-1;
  dma_tx_reset();
  dma_tx_prepare();
  dma_tx_load(tx);
  dma_rx_reset();
  dma_rx_prepare();
  dma_rx_load(rx);
  spi->dev->dma_conf.dma_continue = continuous;
  triggerDMARX();
  triggerDMATX();
}
void DMASPI::startDMA(bool continuous){
  startDMA(dmaDescTX,dmaDescRX,continuous);
}

void DMASPI::stopDMA(){
  spi->dev->dma_conf.dma_continue = 0;
  spi->dev->dma_in_link.start = 0;
  spi->dev->dma_out_link.start = 0;
  spi->dev->dma_in_link.addr = 0;
  spi->dev->dma_out_link.addr = 0;
}

DMASPI::~DMASPI(){
  if(dmaDescTX){
    for(uint32_t i=0;i<txDescCount;i++) dmaDescTX[i].end();
    heap_caps_free(dmaDescTX);
  }
  if(dmaDescRX){
    for(uint32_t i=0;i<rxDescCount;i++) dmaDescRX[i].end();
    heap_caps_free(dmaDescRX);
  }
}
#endif
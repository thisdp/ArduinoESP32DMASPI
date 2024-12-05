#include "ArduinoESP32GDMASPI.h"
#include "esp32-hal-spi.h"

void GDMADesc::begin(uint16_t bufferSize){
  if(bufferSize >= 0xFFF) bufferSize = 0xFFF;
  buffer = (uint8_t *)heap_caps_malloc(bufferSize,MALLOC_CAP_DMA);
  memset(buffer,0,bufferSize);
  size = bufferSize;
  length = bufferSize;
  owner = 1;
  suc_eof = 0;
  err_eof = 0;
}

void GDMADesc::end(){
  if(buffer) heap_caps_free(buffer);
}

GDMASPI::GDMASPI(uint8_t host):SPIHost(host){}

void GDMASPI::begin(int sck, int miso, int mosi, int cs) {
  SPIClass::begin(sck, miso, mosi, cs);
  spi = SPIClass::bus();
  spiAttachSS(spi, 0, cs);
  dmaDescTX = 0;
  dmaDescRX = 0;
  //TX
  static gdma_channel_alloc_config_t dmaChannelConfigTX = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_TX,
    .flags = {
      .reserve_sibling = 0
    }
  };
  gdma_new_channel(&dmaChannelConfigTX, &dmaChannelTX);
  //RX
  static gdma_channel_alloc_config_t dmaChannelConfigRX = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_RX,
    .flags = {
      .reserve_sibling = 0
    }
  };
  gdma_new_channel(&dmaChannelConfigRX, &dmaChannelRX);
  static gdma_strategy_config_t strategyConfig = {
    .owner_check = false,
    .auto_update_desc = false
  };
  gdma_apply_strategy(dmaChannelTX, &strategyConfig);
  gdma_apply_strategy(dmaChannelRX, &strategyConfig);
  static gdma_trigger_t dmaTrigger = {
    .periph = GDMA_TRIG_PERIPH_SPI,
    .instance_id = SPIHost,
  };
  gdma_connect(dmaChannelTX, dmaTrigger);
  gdma_connect(dmaChannelRX, dmaTrigger);
  static gdma_transfer_ability_t ability = {
    .sram_trans_align = 32,
    .psram_trans_align = 64,
  };
  gdma_set_transfer_ability(dmaChannelTX, &ability);
  gdma_set_transfer_ability(dmaChannelRX, &ability);
  spi->dev->dma_conf.dma_afifo_rst = 1;
  spi->dev->dma_conf.buf_afifo_rst = 1;
  spi->dev->dma_conf.rx_afifo_rst = 1;
  spi->dev->dma_conf.rx_eof_en = 1;

  spi->dev->cmd.update = 1;
}

void GDMASPI::setHardwareCSEnabled(bool enabled){
  if(enabled){
    spiSSEnable(spi);
  }else{
    spiSSDisable(spi);
  }
}

void GDMASPI::initDMA(uint32_t txDescs, uint32_t rxDescs, uint16_t dataLen){
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
  dmaDescTX = (GDMADesc*)heap_caps_malloc(sizeof(GDMADesc)*txDescs, MALLOC_CAP_DMA);
  dmaDescRX = (GDMADesc*)heap_caps_malloc(sizeof(GDMADesc)*rxDescs, MALLOC_CAP_DMA);
  for(uint8_t i=0;i<txDescs;i++){
    dmaDescTX[i].begin(dataLen);
    dmaDescTX[i].linkNext(dmaDescTX[(i+1)%txDescs]);
  }
  for(uint8_t i=0;i<rxDescs;i++){
    dmaDescRX[i].begin(dataLen);
    dmaDescRX[i].linkNext(dmaDescRX[(i+1)%rxDescs]);
  }
  dmaDataLength = dataLen;
  spi->dev->cmd.update = 1;
}

void GDMASPI::startDMA(GDMADesc *tx,GDMADesc *rx){
  gdma_start(dmaChannelTX, (intptr_t)tx);
  gdma_start(dmaChannelRX, (intptr_t)rx);
  spi->dev->ms_dlen.ms_data_bitlen = dmaDataLength*8-1;
  spi->dev->dma_conf.dma_tx_ena = 1;
  spi->dev->dma_conf.dma_rx_ena = 1;
  spi->dev->cmd.update = 1;
}

void GDMASPI::startDMA(){
  startDMA(dmaDescTX,dmaDescRX);
}

void GDMASPI::stopDMA(){
  gdma_stop(dmaChannelTX);
  gdma_stop(dmaChannelRX);
  spi->dev->dma_conf.dma_tx_ena = 0;
  spi->dev->dma_conf.dma_rx_ena = 0;
  spi->dev->cmd.update = 1;
}

void GDMASPI::registerCallBack(GDMASPICallBack cb){
  static gdma_rx_event_callbacks_t rx_cbs = {
    .on_recv_eof = cb
  };
  gdma_register_rx_event_callbacks(dmaChannelRX, &rx_cbs, NULL);  // Enable DMA transfer callback
}

GDMASPI::~GDMASPI(){
  if(dmaDescTX){
    for(uint32_t i=0;i<txDescCount;i++) dmaDescTX[i].end();
    heap_caps_free(dmaDescTX);
  }
  if(dmaDescRX){
    for(uint32_t i=0;i<rxDescCount;i++) dmaDescRX[i].end();
    heap_caps_free(dmaDescRX);
  }
}

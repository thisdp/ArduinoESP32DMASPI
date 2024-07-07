#include "ArduinoESP32GDMASPI.h"
#include "esp32-hal-spi.h"

void GDMADesc::begin(uint8_t *inputBuffer, uint16_t bufferSize){
  if(bufferSize >= 0xFFF) bufferSize = 0xFFF;
  buffer = inputBuffer;
  memset(buffer,0,bufferSize);
  size = bufferSize;
  length = bufferSize;
  owner = 1;
  suc_eof = 0;
  err_eof = 0;
}

GDMASPI::GDMASPI(uint8_t host):SPIHost(host){}

void GDMASPI::begin(int sck, int miso, int mosi, int cs) {
  SPIClass::begin(sck, miso, mosi, cs);
  spi = bus();
  spiAttachSS(spi, 0, cs);
  spiSSEnable(spi);
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
    .auto_update_desc = true
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

  
  SPISettings settings(80000000,SPI_MSBFIRST,SPI_MODE0);
  beginTransaction(settings);
}

void GDMASPI::initDMA(uint32_t descs, uint16_t length){
  if(dmaDescTX) delete dmaDescTX;
  if(dmaDescRX) delete dmaDescRX;
  if(dmaBufferTX) delete dmaBufferTX;
  if(dmaBufferRX) delete dmaBufferRX;
  dmaDescTX = (GDMADesc*)heap_caps_malloc(sizeof(GDMADesc)*descs, MALLOC_CAP_INTERNAL);
  dmaDescRX = (GDMADesc*)heap_caps_malloc(sizeof(GDMADesc)*descs, MALLOC_CAP_INTERNAL);
  dmaBufferTX = (uint8_t*)heap_caps_malloc(length*descs, MALLOC_CAP_DMA);
  dmaBufferRX = (uint8_t*)heap_caps_malloc(length*descs, MALLOC_CAP_DMA);
  for(uint8_t i=0;i<descs;i++){
    dmaDescTX[i].begin(&(dmaBufferTX[i*length]),length);
    dmaDescTX[i].linkTo(dmaDescTX[(i+1)%descs]);
    dmaDescRX[i].begin(&(dmaBufferRX[i*length]),length);
    dmaDescRX[i].linkTo(dmaDescRX[(i+1)%descs]);
  }
  
    
  spi->dev->ms_dlen.ms_data_bitlen = length*8-1;
  startDMA();
  spi->dev->cmd.update = 1;
}

void GDMASPI::startDMA(){

  gdma_start(dmaChannelTX, (intptr_t)dmaDescTX);
  gdma_start(dmaChannelRX, (intptr_t)dmaDescRX);
  
  spi->dev->dma_conf.dma_tx_ena = 1;
  spi->dev->dma_conf.dma_rx_ena = 1;
  spi->dev->cmd.update = 1;
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

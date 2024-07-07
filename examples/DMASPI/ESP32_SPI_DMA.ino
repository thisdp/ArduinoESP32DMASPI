constexpr int8_t SPI1CS = 26;
constexpr int8_t SPI1MISO = 33;
constexpr int8_t SPI1MOSI = 32;
constexpr int8_t SPI1SCK = 25;

class TPSCounter{
public:
    uint32_t TPS;
    uint32_t counter;
    uint32_t lastTick;
    TPSCounter(): TPS(0), counter(0), lastTick(0) {}
    inline bool inc(){
      counter ++;
      if(millis() - lastTick >= 1000){
        TPS = counter;
        counter = 0;
        lastTick += 1000;
        return true;
      }
      return false;
    }
};

#include "GDMASPI.h"
DMASPI spiDMA(HSPI);
TPSCounter CoreTPS;

hw_timer_t * adcTimer = NULL;
IRAM_ATTR void onADCSample(){
  spiDMA.triggerTransfer();
}

void setup(){
  disableCore0WDT();
  disableCore1WDT();
  Serial.begin(2000000);

  delay(500);
  Serial.println("");
  delay(500);
  Serial.println("Starting");
  
  spiDMA.begin(SPI1SCK,SPI1MISO,SPI1MOSI,SPI1CS);
  spiDMA.initDMA(4,4);
  for(uint8_t i=0;i<4;i++){
    uint32_t *bufferTX = (uint32_t *)(spiDMA.dmaDescTX[i].getBuffer());
    *bufferTX = i;
    uint32_t *bufferRX = (uint32_t *)(spiDMA.dmaDescRX[i].getBuffer());
    *bufferRX = i;
    Serial.println(((uint32_t)&(spiDMA.dmaDescTX[i])) & 0xFFFFF);
    Serial.println(((uint32_t)&(spiDMA.dmaDescRX[i])) & 0xFFFFF);
  }
  spiDMA.startDMA(true);
  adcTimer = timerBegin(2,80,true);
  timerAttachInterrupt(adcTimer, onADCSample,false); //开启定时器中断
  timerAlarmWrite(adcTimer, 8, true);  //100K
  spiDMA.triggerTransfer();
  //timerAlarmEnable(adcTimer);
  Serial.println("Started");
}

void loop(){
  while(1){
    if(CoreTPS.inc()){
      //Serial.println(CoreTPS.TPS);
    }
  }

  //delay(1);
  /*for(uint8_t i=0;i<4;i++){
    Serial.print(i);
    Serial.print(":");
    Serial.println(rxDesc[i].suc_eof);
  }*/
}
constexpr int8_t SPI1CS = 45;
constexpr int8_t SPI1MISO = 46;
constexpr int8_t SPI1MOSI = 37;
constexpr int8_t SPI1SCK = 38;
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
GDMASPI spiDMA(FSPI);
TPSCounter CoreTPS;

hw_timer_t * adcTimer = NULL;
IRAM_ATTR void onADCSample(){
  spiDMA.triggerTransfer();
}

void setup(){
  disableCore0WDT();
  disableCore1WDT();
  Serial.begin(2000000);

  delay(2000);
  Serial.println("Starting");
  
  spiDMA.begin(SPI1SCK,SPI1MISO,SPI1MOSI,SPI1CS);
  spiDMA.initDMA(4,4);
  adcTimer = timerBegin(2,80,true);
  timerAttachInterrupt(adcTimer, onADCSample,false); //开启定时器中断
  timerAlarmWrite(adcTimer, 10, true);  //100K
  //timerAlarmEnable(adcTimer);
  Serial.println("Started");
}

void loop(){
  while(1){
    delay(100);
    spiDMA.triggerTransfer();
    if(CoreTPS.inc()){
      Serial.println(CoreTPS.TPS);
    }
  }

  //delay(1);
  /*for(uint8_t i=0;i<4;i++){
    Serial.print(i);
    Serial.print(":");
    Serial.println(rxDesc[i].suc_eof);
  }*/
}
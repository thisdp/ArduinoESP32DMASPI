#include "ArduinoESP32DMADesc.h"
#include <string.h> // For memset

int alignRound(int value, int alignment) {
    // 如果value已经是alignment的倍数，则直接返回value
    if (value % alignment == 0) {
        return value;
    }
    // 否则，计算并返回下一个alignment的倍数
    return ((value / alignment) + 1) * alignment;
}

DMADesc::DMADesc() : descWord(0), buffer(nullptr), next(nullptr) {}

DMADesc::~DMADesc() { end(); }

void DMADesc::end() { 
    if(buffer != nullptr) heap_caps_free(buffer); 
    buffer = nullptr; 
};

uint16_t DMADesc::begin(uint16_t bufferSize){
    end();  //Clear previous if exists
    if(bufferSize >= 0xFFF) bufferSize = 0xFFF;
    
    buffer = (uint8_t *)heap_caps_aligned_alloc(4,bufferSize,MALLOC_CAP_DMA);
    if(buffer == nullptr) return 0;
    memset(buffer,0,bufferSize);
    size = bufferSize;
    length = bufferSize;
    setOwnedByDMA(true);
    setLast(false);
    clearError();
    return bufferSize;
}

bool DMADesc::hasError(){ return err_eof; }
void DMADesc::clearError() { err_eof = 0; }
bool DMADesc::isLast(){ return suc_eof; }
void DMADesc::setLast(bool isLast){ suc_eof = isLast; }
bool DMADesc::isOwnedByDMA(){ return owner; }
void DMADesc::setOwnedByDMA(bool ownedByDMA){ owner = ownedByDMA; }
void DMADesc::linkNext(DMADesc* nextDMADesc){ next = nextDMADesc; }
void DMADesc::linkNext(DMADesc& nextDMADesc){ next = &nextDMADesc; }
DMADesc* DMADesc::getNext() { return next; }
uint8_t* DMADesc::getBuffer() { return (uint8_t*)buffer; }
bool DMADesc::hasBuffer() { return buffer != nullptr; }
void DMADesc::clear() {
	descWord = 0;
	buffer = 0;
	next = 0;
}

DMADescManager::~DMADescManager() {
    clearDescs();
}

bool DMADescManager::allocDescs(uint32_t descCount) {
    DMADesc* desc = static_cast<DMADesc*>(heap_caps_malloc(sizeof(DMADesc)*descCount, MALLOC_CAP_DMA));
    if (desc == nullptr) return false;  //Fail
    for (uint32_t i = 0; i < descCount; ++i) {
        new(&(desc[i])) DMADesc(); // Placement new
        dmaDescs.push_back(desc);
    }
    return true;
}

bool DMADescManager::initDesc(uint32_t index, uint16_t bufferSize){
    if(index >= dmaDescs.size()) return false;
    if(dmaDescs[index]->begin(bufferSize) == 0) return false;
    return true;
}

DMADesc *DMADescManager::getDesc(uint32_t index) { 
    if(index >= dmaDescs.size()) return nullptr;
    return dmaDescs[index];
}

void DMADescManager::clearDescs() {
    for (auto* desc : dmaDescs) {
        if (desc == nullptr) continue;
        desc->~DMADesc();
        heap_caps_free(desc);
    }
    dmaDescs.clear();
}


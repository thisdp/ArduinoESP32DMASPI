#pragma once
#include <vector>
#include "Arduino.h"
using namespace std;

#pragma pack(push, 1)

class DMADesc {
public:
    DMADesc();
    ~DMADesc();
    void end();
    uint16_t begin(uint16_t bufferSize);
    bool hasError();
    void clearError();
    bool isLast();
    void setLast(bool isLast);
    bool isOwnedByDMA();
    void setOwnedByDMA(bool ownedByDMA);
    void linkNext(DMADesc* nextDMADesc);
    void linkNext(DMADesc& nextDMADesc);
    DMADesc* getNext();
    uint8_t* getBuffer();
    bool hasBuffer();
private:
    union {
        struct {
            uint32_t size : 12;         /*!< Buffer size */
            uint32_t length : 12;       /*!< Number of valid bytes in the buffer */
            uint32_t reversed24_27 : 4; /*!< Reserved */
            uint32_t err_eof : 1;       /*!< Whether the received buffer contains error */
            uint32_t reserved29 : 1;    /*!< Reserved */
            uint32_t suc_eof : 1;       /*!< Whether the descriptor is the last one in the link */
            uint32_t owner : 1;         /*!< Who is allowed to access the buffer that this descriptor points to */
        };                          /*!< Descriptor Word 0 */
        uint32_t descWord;
    };
    void* buffer;                   /*!< Pointer to the buffer */
    DMADesc* next;                  /*!< Pointer to the next descriptor (set to NULL if the descriptor is the last one, e.g. suc_eof=1) */
};

#pragma pack(pop)

class DMADescManager {  //DMA Description Manager，防止难受
public:
    DMADescManager() = default;
    ~DMADescManager();

    bool initDesc(uint32_t index, uint16_t bufferSize);
    DMADesc* getDesc(uint32_t index);
    bool allocDescs(uint32_t descCount);
    void clearDescs();

private:
    vector<DMADesc*> dmaDescs;
};



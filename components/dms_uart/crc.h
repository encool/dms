#pragma once

#include <cstdint>

class CRC8 {
public:
    CRC8(uint8_t poly, uint8_t init, uint8_t xorout, bool refin, bool refout);
    void restart();
    void add(uint8_t data);
    uint8_t calc();

private:
    uint8_t poly_;
    uint8_t init_;
    uint8_t xorout_;
    bool refin_;
    bool refout_;
    uint8_t crc_;
};

class CRC16 {
public:
    CRC16(uint16_t poly, uint16_t init, uint16_t xorout, bool refin, bool refout);
    void restart();
    void add(uint8_t data);
    uint16_t calc();

private:
    uint16_t poly_;
    uint16_t init_;
    uint16_t xorout_;
    bool refin_;
    bool refout_;
    uint16_t crc_;
};
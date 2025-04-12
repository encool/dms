#include "crc.h"

CRC8::CRC8(uint8_t poly, uint8_t init, uint8_t xorout, bool refin, bool refout)
    : poly_(poly), init_(init), xorout_(xorout), refin_(refin), refout_(refout) {
    restart();
}

void CRC8::restart() {
    crc_ = init_;
}

void CRC8::add(uint8_t data) {
    if (refin_) {
        data = (data >> 4) | (data << 4);
        data = ((data & 0xCC) >> 2) | ((data & 0x33) << 2);
        data = ((data & 0xAA) >> 1) | ((data & 0x55) << 1);
    }
    
    crc_ ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc_ & 0x80) {
            crc_ = (crc_ << 1) ^ poly_;
        } else {
            crc_ <<= 1;
        }
    }
}

uint8_t CRC8::calc() {
    if (refout_) {
        crc_ = (crc_ >> 4) | (crc_ << 4);
        crc_ = ((crc_ & 0xCC) >> 2) | ((crc_ & 0x33) << 2);
        crc_ = ((crc_ & 0xAA) >> 1) | ((crc_ & 0x55) << 1);
    }
    return crc_ ^ xorout_;
}

CRC16::CRC16(uint16_t poly, uint16_t init, uint16_t xorout, bool refin, bool refout)
    : poly_(poly), init_(init), xorout_(xorout), refin_(refin), refout_(refout) {
    restart();
}

void CRC16::restart() {
    crc_ = init_;
}

void CRC16::add(uint8_t data) {
    if (refin_) {
        data = (data >> 4) | (data << 4);
        data = ((data & 0xCC) >> 2) | ((data & 0x33) << 2);
        data = ((data & 0xAA) >> 1) | ((data & 0x55) << 1);
    }
    
    crc_ ^= (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc_ & 0x8000) {
            crc_ = (crc_ << 1) ^ poly_;
        } else {
            crc_ <<= 1;
        }
    }
}

uint16_t CRC16::calc() {
    if (refout_) {
        crc_ = (crc_ >> 8) | (crc_ << 8);
    }
    return crc_ ^ xorout_;
}
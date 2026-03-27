#include "I2C_Class.h"

void I2C_Class::begin(TwoWire* wire, uint8_t sda, uint8_t scl, long freq) {
    _wire = wire;
    _sda  = sda;
    _scl  = scl;
    _freq = freq;
    // Wire.begin() déjà appelé dans setup(), on ne le réinitialise pas
}

bool I2C_Class::exist(uint8_t addr) {
    _wire->beginTransmission(addr);
    return _wire->endTransmission() == 0;
}

bool I2C_Class::writeBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    for (uint8_t i = 0; i < length; i++) _wire->write(buffer[i]);
    return _wire->endTransmission() == 0;
}

bool I2C_Class::readBytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    _wire->requestFrom(addr, length);
    for (uint8_t i = 0; i < length; i++) buffer[i] = _wire->read();
    return true;
}

bool I2C_Class::readU16(uint8_t addr, uint8_t reg_addr, uint16_t* value) {
    uint8_t buf[2];
    if (!readBytes(addr, reg_addr, buf, 2)) return false;
    *value = (buf[0] << 8) | buf[1];
    return true;
}

bool I2C_Class::writeU16(uint8_t addr, uint8_t reg_addr, uint16_t value) {
    uint8_t buf[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return writeBytes(addr, reg_addr, buf, 2);
}

bool I2C_Class::writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
    return writeBytes(addr, reg, &data, 1);
}

uint8_t I2C_Class::readByte(uint8_t addr, uint8_t reg) {
    uint8_t data = 0;
    readBytes(addr, reg, &data, 1);
    return data;
}

bool I2C_Class::writeBitOn(uint8_t addr, uint8_t reg, uint8_t bit) {
    uint8_t data = readByte(addr, reg);
    data |= bit;
    return writeByte(addr, reg, data);
}

bool I2C_Class::writeBitOff(uint8_t addr, uint8_t reg, uint8_t bit) {
    uint8_t data = readByte(addr, reg);
    data &= ~bit;
    return writeByte(addr, reg, data);
}

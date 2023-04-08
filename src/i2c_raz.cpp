#include "i2c_raz.h"
#include <Arduino.h>
#include <Wire.h>

int write_data(uint8_t addr, uint8_t* dst, size_t len)
{
    Wire.beginTransmission(addr);
    int l = Wire.write(dst, len);
    Wire.endTransmission();
    return l;
}

int read_data(uint8_t addr, uint8_t* dst, size_t len)
{
    size_t idx = 0;
    uint8_t value;
    Wire.requestFrom(addr, len);
    while (Wire.available()) {
        value = Wire.read();
        if (idx < len) {
            dst[idx] = value;
            idx++;
        }
    }
    return idx;
}

/* Convenience functions for various common i2c operations */
void reg_write_uint8(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = { reg, value };
    write_data(address, buffer, 2);
}

uint8_t reg_read_uint8(uint8_t address, uint8_t reg)
{
    uint8_t value;
    write_data(address, &reg, 1);
    read_data(address, (uint8_t*)&value, sizeof(uint8_t));
    return value;
}

uint16_t reg_read_uint16(uint8_t address, uint8_t reg)
{
    uint16_t value;
    write_data(address, &reg, 1);
    read_data(address, (uint8_t*)&value, sizeof(uint16_t));
    return value;
}

uint32_t reg_read_uint32(uint8_t address, uint8_t reg)
{
    uint32_t value;
    write_data(address, &reg, 1);
    read_data(address, (uint8_t*)&value, sizeof(uint32_t));
    return value;
}

int16_t reg_read_int16(uint8_t address, uint8_t reg)
{
    int16_t value;
    write_data(address, &reg, 1);
    read_data(address, (uint8_t*)&value, sizeof(int16_t));
    return value;
}

int write_bytes(uint8_t address, uint8_t reg, const uint8_t* buf, int len)
{
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    for (int x = 0; x < len; x++) {
        buffer[x + 1] = buf[x];
    }
    return write_data(address, buffer, len + 1);
};

int read_bytes(uint8_t address, uint8_t reg, uint8_t* buf, int len)
{
    write_data(address, &reg, 1);
    read_data(address, buf, len);
    return len;
};

uint8_t get_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask)
{
    uint8_t value;
    read_bytes(address, reg, &value, 1);
    return value & (mask << shift);
}

void set_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask)
{
    uint8_t value;
    read_bytes(address, reg, &value, 1);
    value |= mask << shift;
    write_bytes(address, reg, &value, 1);
}

void clear_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask)
{
    uint8_t value;
    read_bytes(address, reg, &value, 1);
    value &= ~(mask << shift);
    write_bytes(address, reg, &value, 1);
}

uint8_t get_bits(uint8_t address, uint8_t reg, uint8_t shift)
{
    uint8_t value;
    read_bytes(address, reg, &value, 1);
    return value & (0b1 << shift);
}

void set_bits(uint8_t address, uint8_t reg, uint8_t shift)
{
    uint8_t value;
    read_bytes(address, reg, &value, 1);
    value |= 0b1 << shift;
    write_bytes(address, reg, &value, 1);
}

void clear_bits(uint8_t address, uint8_t reg, uint8_t shift)
{
    uint8_t value;
    read_bytes(address, reg, &value, 1);
    value &= ~(0b1 << shift);
    write_bytes(address, reg, &value, 1);
}
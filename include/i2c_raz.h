#include <Arduino.h>

/* Small i2c functions to read and write registers from/to a sensor */

int write_data(uint8_t addr, uint8_t* dst, size_t len);
int read_data(uint8_t addr, uint8_t* dst, size_t len);

/* Convenience functions for various common i2c operations */
void reg_write_uint8(uint8_t address, uint8_t reg, uint8_t value);
uint8_t reg_read_uint8(uint8_t address, uint8_t reg);
uint16_t reg_read_uint16(uint8_t address, uint8_t reg);
uint32_t reg_read_uint32(uint8_t address, uint8_t reg);
int16_t reg_read_int16(uint8_t address, uint8_t reg);
int write_bytes(uint8_t address, uint8_t reg, const uint8_t* buf, int len);
int read_bytes(uint8_t address, uint8_t reg, uint8_t* buf, int len);
uint8_t get_bits(uint8_t address, uint8_t reg, uint8_t shift);
void set_bits(uint8_t address, uint8_t reg, uint8_t shift);
void clear_bits(uint8_t address, uint8_t reg, uint8_t shift);
uint8_t get_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask);
void set_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask);
void clear_bits(uint8_t address, uint8_t reg, uint8_t shift, uint8_t mask);
#pragma once

void i2c_init();
int i2c_write(uint8_t addr, const uint8_t *data, size_t len);
int i2c_write_partial(uint8_t addr, const uint8_t *data, size_t len, bool stop);
int i2c_read(uint8_t addr, uint8_t *data, size_t len);
int i2c_write_read(uint8_t addr, const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen);

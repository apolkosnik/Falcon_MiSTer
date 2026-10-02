// Simulation shim of Main_MiSTer spi.h: the sector transfer calls used by
// support/falcon.  spi_block_write hands a block to the FPGA model,
// spi_block_read takes the block the FPGA model collected.
#ifndef SPI_H
#define SPI_H
#include <stdint.h>

void EnableIO();
void DisableIO();
uint16_t spi_w(uint16_t word);
void spi_block_read(uint8_t *addr, int wide, int sz = 512);
void spi_block_write(const uint8_t *addr, int wide, int sz = 512);

#endif

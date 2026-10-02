// Simulation shim of Main_MiSTer user_io.h: core detection and the sector
// transfer command codes used by support/falcon.
#ifndef USER_IO_H
#define USER_IO_H
#include <stdint.h>

#define UIO_SECTOR_RD   0x17  // SD card sector read
#define UIO_SECTOR_WR   0x18  // SD card sector write

char is_falcon();
int user_io_get_width();

#endif

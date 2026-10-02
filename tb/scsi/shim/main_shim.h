// main_shim.h - the testbench's view of the simulated Main_MiSTer: the SPI
// capture of the shims and a model of user_io's sector service loop and
// file mount (user_io_file_mount / the sd request handling), calling the
// real support/falcon hooks exactly where user_io.cpp calls them.
#pragma once
#include <stdint.h>
#include "file_io.h"

extern bool     g_is_falcon;     // core name "Falcon" (Falcon Main) or not (stock Main)
extern int      g_spi_errors;

struct MainSim
{
	fileTYPE sd_image[16];
	bool     cangrow[16] = {};
	uint64_t ops_hook = 0, ops_generic = 0;

	// user_io_file_mount: returns the image size given to the core (0 = no image)
	uint64_t mount(int index, const char *name, bool *ro);
	void     unmount(int index);
	// the sd request loop for one 512-byte block (op 1 = read, 2 = write)
	void     read_block(int disk, uint32_t lba, uint8_t *buf);
	void     write_block(int disk, uint32_t lba, const uint8_t *buf);
};

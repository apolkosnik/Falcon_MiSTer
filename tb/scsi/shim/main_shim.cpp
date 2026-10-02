// main_shim.cpp - implementation of the Main_MiSTer shims (file_io, spi,
// user_io) for the SCSI bench, and the user_io model (mount and sd request
// loop) around the real support/falcon/falcon_scsi.cpp.
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include "file_io.h"
#include "spi.h"
#include "user_io.h"
#include "main_shim.h"
#include "support/falcon/falcon_scsi.h"

bool g_is_falcon = true;
int  g_spi_errors = 0;

// ---------------- file_io ----------------
fileTYPE::fileTYPE() : filp(0), mode(0), type(0), zip(0), size(0), offset(0) { path[0] = 0; name[0] = 0; }
fileTYPE::~fileTYPE() { if (filp) fclose(filp); }
int fileTYPE::opened() { return filp != 0; }

int FileOpenEx(fileTYPE *file, const char *name, int mode, char mute, int use_zip)
{
	(void)mute; (void)use_zip;
	if (file->filp) FileClose(file);
	const char *m = ((mode & O_RDWR) || (mode & O_WRONLY)) ? "r+b" : "rb";
	file->filp = fopen(name, m);
	if (!file->filp) { file->size = 0; return 0; }
	fseeko64(file->filp, 0, SEEK_END);
	file->size = ftello64(file->filp);
	fseeko64(file->filp, 0, SEEK_SET);
	file->offset = 0;
	file->mode = mode;
	snprintf(file->path, sizeof(file->path), "%s", name);
	const char *s = strrchr(name, '/');
	snprintf(file->name, sizeof(file->name), "%s", s ? s + 1 : name);
	return 1;
}
int FileOpen(fileTYPE *file, const char *name, char mute) { return FileOpenEx(file, name, O_RDONLY, mute); }
int FileClose(fileTYPE *file)
{
	if (file->filp) fclose(file->filp);
	file->filp = 0;
	file->zip = 0;
	return 1;
}
int FileSeek(fileTYPE *file, __off64_t offset, int origin)
{
	if (!file->filp) return 0;
	if (fseeko64(file->filp, offset, origin) < 0) return 0;
	file->offset = ftello64(file->filp);
	return 1;
}
int FileReadAdv(fileTYPE *file, void *pBuffer, int length, int failres)
{
	if (!file->filp) return failres;
	size_t r = fread(pBuffer, 1, length, file->filp);
	if ((int)r != length && ferror(file->filp)) { clearerr(file->filp); return failres; }
	file->offset += r;
	return (int)r;
}
int FileWriteAdv(fileTYPE *file, void *pBuffer, int length, int failres)
{
	if (!file->filp) return failres;
	size_t r = fwrite(pBuffer, 1, length, file->filp);
	fflush(file->filp);
	if ((int)r != length) return failres;
	file->offset += r;
	return (int)r;
}
int FileCanWrite(const char *name) { return access(name, W_OK) == 0; }

// ---------------- spi ----------------
static int      io_on = 0;
static uint16_t spi_cmd = 0;
static uint8_t  to_fpga[512], from_fpga[512];
static int      to_fpga_valid = 0;

void EnableIO() { if (io_on) { printf("SHIM: EnableIO twice\n"); g_spi_errors++; } io_on = 1; spi_cmd = 0; }
void DisableIO() { if (!io_on) { printf("SHIM: DisableIO without EnableIO\n"); g_spi_errors++; } io_on = 0; }
uint16_t spi_w(uint16_t w) { if (!io_on) { printf("SHIM: spi_w outside IO\n"); g_spi_errors++; } spi_cmd = w; return 0; }
void spi_block_write(const uint8_t *addr, int wide, int sz)
{
	(void)wide;
	if (!io_on || (spi_cmd & 0xFF) != 0x17 || sz != 512) { printf("SHIM: bad sector read transfer (cmd %04x sz %d)\n", spi_cmd, sz); g_spi_errors++; }
	memcpy(to_fpga, addr, 512);
	to_fpga_valid = 1;
}
void spi_block_read(uint8_t *addr, int wide, int sz)
{
	(void)wide;
	if (!io_on || (spi_cmd & 0xFF) != 0x18 || sz != 512) { printf("SHIM: bad sector write transfer (cmd %04x sz %d)\n", spi_cmd, sz); g_spi_errors++; }
	memcpy(addr, from_fpga, 512);
}

// ---------------- user_io ----------------
char is_falcon() { return g_is_falcon ? 1 : 0; }
int user_io_get_width() { return 0; }

// user_io_file_mount (the generic open + the hooks in the order of user_io.cpp)
uint64_t MainSim::mount(int index, const char *name, bool *ro)
{
	int writable = 0, ret = 0;
	cangrow[index] = false;
	if (name && name[0])
	{
		writable = FileCanWrite(name);
		ret = FileOpenEx(&sd_image[index], name, writable ? (O_RDWR | O_SYNC) : O_RDONLY);
		if (ret) ret = falcon_scsi_mount_hook(index, name, &sd_image[index], &writable);
	}
	else
	{
		FileClose(&sd_image[index]);
		falcon_scsi_unmount(index);
	}
	if (!ret) { sd_image[index].size = 0; writable = 0; }
	*ro = !writable;
	return (uint64_t)sd_image[index].size;
}
void MainSim::unmount(int index) { bool ro; mount(index, "", &ro); }

// the sd request loop: hook first, then the generic path of user_io.cpp
void MainSim::read_block(int disk, uint32_t lba, uint8_t *buf)
{
	int ack = (disk + 1) << 8;
	to_fpga_valid = 0;
	int r = falcon_sd_service(disk, 1, lba, 512, ack);
	if (r > 0)
	{
		if (!to_fpga_valid) { printf("SHIM: hook served a read without a block\n"); g_spi_errors++; }
		memcpy(buf, to_fpga, 512);
		ops_hook++;
		return;
	}
	if (r < 0) { printf("SHIM: hook refused a read\n"); g_spi_errors++; }
	// generic: read from the image, an empty block after an error
	ops_generic++;
	fileTYPE *f = &sd_image[disk];
	int done = 0;
	memset(buf, 0, 512);
	if (f->size && FileSeek(f, (__off64_t)lba * 512, SEEK_SET) && FileReadAdv(f, buf, 512) > 0) done = 1;
	if (!done) memset(buf, 0, 512);
}
void MainSim::write_block(int disk, uint32_t lba, const uint8_t *buf)
{
	int ack = (disk + 1) << 8;
	memcpy(from_fpga, buf, 512);
	int r = falcon_sd_service(disk, 2, lba, 512, ack);
	if (r > 0) { ops_hook++; return; }
	if (r < 0) { printf("SHIM: hook refused a write\n"); g_spi_errors++; }
	ops_generic++;
	fileTYPE *f = &sd_image[disk];
	uint64_t size = f->size / 512;
	int sz = 512;
	if (lba <= size && FileSeek(f, (__off64_t)lba * 512, SEEK_SET))
	{
		if (!cangrow[disk])
		{
			__off64_t rem = f->size - f->offset;
			sz = (rem >= sz) ? sz : (int)rem;
		}
		if (sz > 0) FileWriteAdv(f, (void *)buf, sz);
	}
}

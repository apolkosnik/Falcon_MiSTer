// Simulation shim of Main_MiSTer file_io.h: only what support/falcon uses.
// Same names and signatures as the real header; the files are plain host
// files opened with stdio.
#ifndef _FAT16_H_INCLUDED
#define _FAT16_H_INCLUDED

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include "spi.h"

struct fileTYPE
{
	fileTYPE();
	~fileTYPE();
	int opened();

	FILE      *filp;
	int        mode;
	int        type;
	void      *zip;
	__off64_t  size;
	__off64_t  offset;
	char       path[1024];
	char       name[261];
};

int  FileOpenEx(fileTYPE *file, const char *name, int mode, char mute = 0, int use_zip = 1);
int  FileOpen(fileTYPE *file, const char *name, char mute = 0);
int  FileClose(fileTYPE *file);
int  FileSeek(fileTYPE *file, __off64_t offset, int origin);
int  FileReadAdv(fileTYPE *file, void *pBuffer, int length, int failres = 0);
int  FileWriteAdv(fileTYPE *file, void *pBuffer, int length, int failres = 0);
int  FileCanWrite(const char *name);

#endif

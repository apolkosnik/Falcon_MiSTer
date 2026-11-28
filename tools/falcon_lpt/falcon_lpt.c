/*
 * falcon_lpt: the HPS side of the core's printer port (rtl/falcon/falcon_lpt.sv).
 *
 * The core puts every byte the machine strobes out of its Centronics port in a ring
 * in DDR3 at 0x30EB8000 (guest $EB8000); 16-bit words in guest (big endian) byte order:
 *   +$0000 MAGIC $4C50 ("LP", written here)   +$0002 HEARTBEAT (here, ten a second)
 *   +$0004 HEAD (core): bytes put in, a free-running count
 *   +$0006 TAIL (here): bytes taken out
 *   +$0008 STATUS (here): bit 0 = the printer cannot take data now (the core's BUSY)
 *   +$0800 + 2*n ring entry n (0..2047), the byte in the low half
 * While this program runs the machine sees a printer; without it, one switched off.
 *
 * The bytes go, unchanged, to the first of these that is there:
 *   - a USB printer or USB-to-parallel cable (USB class 7), driven through usbfs: the
 *     MiSTer kernel has no usblp.  Its status byte (paper out, off line, error) holds
 *     the core's BUSY high;
 *   - -n ADDRESS[:PORT]: a network printer's raw port (9100) at an IPv4 address, one
 *     connection a job;
 *   - a file a job in DIR (-o, default /media/fat/printer): print-YYYYMMDD-HHMMSS.prn.
 * A job ends after -t SECONDS (default 5) without a byte.
 *
 *   falcon_lpt [-o DIR] [-n ADDRESS[:PORT]] [-t SECONDS] [-U] [-c CPU] [-v]
 *     -U           do not look for a USB printer
 *   falcon_lpt -m FILE ...          map FILE instead of /dev/mem (testing)
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>

#define LPT_PHYS   0x30EB8000u
#define LPT_SIZE   0x2000u
#define MAGIC      0x4C50
#define HEARTBEAT  0x02
#define HEAD       0x04
#define TAIL       0x06
#define STATUS     0x08
#define RING       0x800u
#define RING_N     2048u

static volatile uint8_t *mb;
static int verbose;
static volatile sig_atomic_t quit;

static unsigned rd16(unsigned o) { return mb[o] << 8 | mb[o + 1]; }
static void wr16(unsigned o, unsigned v) { mb[o] = v >> 8; mb[o + 1] = v; }
static void on_signal(int s) { (void)s; quit = 1; }

static double now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec / 1e9;
}

/* ---- the sinks: each takes up to n bytes and returns how many went (0: not now) ---- */

/* a file a job */
static const char *outdir = "/media/fat/printer";
static int file_fd = -1;

static int file_put(const uint8_t *p, int n)
{
	if (file_fd < 0) {
		char name[512], stamp[32];
		time_t t = time(NULL);
		strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", localtime(&t));
		mkdir(outdir, 0777);
		for (int k = 0; file_fd < 0 && k < 100; k++) {
			if (k) snprintf(name, sizeof name, "%s/print-%s-%d.prn", outdir, stamp, k);
			else snprintf(name, sizeof name, "%s/print-%s.prn", outdir, stamp);
			file_fd = open(name, O_WRONLY | O_CREAT | O_EXCL, 0666);
			if (file_fd < 0 && errno != EEXIST) break;
		}
		if (file_fd < 0) { if (verbose) perror("falcon_lpt: output file"); return 0; }
		if (verbose) fprintf(stderr, "falcon_lpt: job to %s\n", name);
	}
	int w = write(file_fd, p, n);
	return w > 0 ? w : 0;
}

static void file_end(void)
{
	if (file_fd >= 0) { fsync(file_fd); close(file_fd); file_fd = -1; }
}

/* a network printer's raw port */
static char net_host[256], net_port[16] = "9100";
static int net_fd = -1;
static double net_retry;

static int net_put(const uint8_t *p, int n)
{
	if (net_fd < 0) {
		if (now() < net_retry) return 0;
		net_retry = now() + 2;
		struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(atoi(net_port)) };
		if (inet_pton(AF_INET, net_host, &sa.sin_addr) != 1) return 0;
		int s = socket(AF_INET, SOCK_STREAM, 0);
		if (s < 0) return 0;
		struct timeval tv = { 2, 0 };
		setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
		if (connect(s, (struct sockaddr *)&sa, sizeof sa) == 0) net_fd = s;
		else close(s);
		if (net_fd < 0) return 0;
		if (verbose) fprintf(stderr, "falcon_lpt: job to %s:%s\n", net_host, net_port);
	}
	int w = send(net_fd, p, n, MSG_NOSIGNAL);
	if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) { close(net_fd); net_fd = -1; }
	return w > 0 ? w : 0;
}

static void net_end(void)
{
	if (net_fd >= 0) { shutdown(net_fd, SHUT_WR); close(net_fd); net_fd = -1; }
}

/* a USB printer-class device through usbfs */
static int usb_fd = -1, usb_if, usb_ep;
static double usb_scan;

static int sysfs_hex(const char *dir, const char *name)
{
	char path[600], buf[32];
	snprintf(path, sizeof path, "%s/%s", dir, name);
	int fd = open(path, O_RDONLY);
	if (fd < 0) return -1;
	int n = read(fd, buf, sizeof buf - 1);
	close(fd);
	if (n <= 0) return -1;
	buf[n] = 0;
	return strtol(buf, NULL, 16);
}

static int sysfs_dec(const char *dir, const char *name)
{
	char path[600], buf[32];
	snprintf(path, sizeof path, "%s/%s", dir, name);
	int fd = open(path, O_RDONLY);
	if (fd < 0) return -1;
	int n = read(fd, buf, sizeof buf - 1);
	close(fd);
	if (n <= 0) return -1;
	buf[n] = 0;
	return atoi(buf);
}

/* the first interface of class 7 with a bulk OUT endpoint */
static void usb_find(void)
{
	static const char *root = "/sys/bus/usb/devices";
	DIR *d = opendir(root);
	if (!d) return;
	struct dirent *e;
	while (usb_fd < 0 && (e = readdir(d))) {
		char *colon = strchr(e->d_name, ':');
		if (!colon) continue;                               /* interfaces are BUS-PORT:CFG.IF */
		char idir[400], ddir[400];
		snprintf(idir, sizeof idir, "%s/%s", root, e->d_name);
		if (sysfs_hex(idir, "bInterfaceClass") != 7) continue;
		int ep = -1;
		DIR *id = opendir(idir);
		struct dirent *ie;
		while (id && (ie = readdir(id))) {
			if (strncmp(ie->d_name, "ep_", 3)) continue;
			char edir[700];
			snprintf(edir, sizeof edir, "%s/%s", idir, ie->d_name);
			int addr = sysfs_hex(edir, "bEndpointAddress"), attr = sysfs_hex(edir, "bmAttributes");
			if (addr >= 0 && !(addr & 0x80) && (attr & 3) == 2) ep = addr;
		}
		if (id) closedir(id);
		if (ep < 0) continue;
		snprintf(ddir, sizeof ddir, "%s/%.*s", root, (int)(colon - e->d_name), e->d_name);
		int bus = sysfs_dec(ddir, "busnum"), dev = sysfs_dec(ddir, "devnum");
		int ifn = sysfs_hex(idir, "bInterfaceNumber");
		if (bus < 0 || dev < 0 || ifn < 0) continue;
		char node[64];
		snprintf(node, sizeof node, "/dev/bus/usb/%03d/%03d", bus, dev);
		int fd = open(node, O_RDWR);
		if (fd < 0) continue;
		struct usbdevfs_ioctl disc = { .ifno = ifn, .ioctl_code = USBDEVFS_DISCONNECT };
		ioctl(fd, USBDEVFS_IOCTL, &disc);                    /* a kernel driver, if one has it */
		if (ioctl(fd, USBDEVFS_CLAIMINTERFACE, &ifn) < 0) { close(fd); continue; }
		usb_fd = fd; usb_if = ifn; usb_ep = ep;
		if (verbose) fprintf(stderr, "falcon_lpt: USB printer %s interface %d endpoint %02x\n", node, ifn, ep);
	}
	closedir(d);
}

static void usb_drop(void)
{
	if (usb_fd >= 0) { close(usb_fd); usb_fd = -1; if (verbose) fprintf(stderr, "falcon_lpt: USB printer gone\n"); }
}

/* GET_PORT_STATUS: bit 5 paper empty, bit 4 selected, bit 3 no error.  A device that
   does not answer is taken as ready. */
static int usb_ready(void)
{
	uint8_t st = 0x18;
	struct usbdevfs_ctrltransfer c = { .bRequestType = 0xA1, .bRequest = 1, .wValue = 0,
		.wIndex = usb_if, .wLength = 1, .timeout = 500, .data = &st };
	if (ioctl(usb_fd, USBDEVFS_CONTROL, &c) < 0) {
		if (errno == ENODEV) { usb_drop(); return 0; }
		return 1;
	}
	return (st & 0x38) == 0x18;
}

static int usb_put(const uint8_t *p, int n)
{
	struct usbdevfs_bulktransfer b = { .ep = usb_ep, .len = n, .timeout = 1000, .data = (void *)p };
	int w = ioctl(usb_fd, USBDEVFS_BULK, &b);
	if (w < 0) {
		if (errno == ENODEV || errno == ESHUTDOWN) usb_drop();
		return 0;
	}
	return w;
}

int main(int argc, char **argv)
{
	const char *memfile = NULL;
	int c, cpu = -1, use_usb = 1;
	double idle = 5;

	while ((c = getopt(argc, argv, "o:n:t:Uc:m:v")) != -1) {
		switch (c) {
		case 'o': outdir = optarg; break;
		case 'n': {
			snprintf(net_host, sizeof net_host, "%s", optarg);
			char *p = strrchr(net_host, ':');
			if (p) { *p = 0; snprintf(net_port, sizeof net_port, "%s", p + 1); }
			break;
		}
		case 't': idle = atof(optarg); break;
		case 'U': use_usb = 0; break;
		case 'c': cpu = atoi(optarg); break;
		case 'm': memfile = optarg; break;
		case 'v': verbose++; break;
		default:
			fprintf(stderr, "usage: falcon_lpt [-o DIR] [-n ADDRESS[:PORT]] [-t SECONDS] [-U] [-c CPU] [-m FILE] [-v]\n");
			return 2;
		}
	}
	if (cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(cpu, &set);
		sched_setaffinity(0, sizeof set, &set);
	}
	int mfd = memfile ? open(memfile, O_RDWR) : open("/dev/mem", O_RDWR | O_SYNC);
	if (mfd < 0) { perror("falcon_lpt: open memory"); return 1; }
	mb = mmap(NULL, LPT_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, memfile ? 0 : LPT_PHYS);
	if (mb == MAP_FAILED) { perror("falcon_lpt: mmap"); return 1; }
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	/* the core reads HEAD back when it sees us: start with an empty ring.  Memory that
	   never held the ring has no MAGIC and gets both counts from here. */
	unsigned tail;
	if (rd16(0) == MAGIC) tail = rd16(HEAD);
	else { tail = 0; wr16(HEAD, 0); }
	wr16(TAIL, tail);
	wr16(STATUS, 0);
	wr16(0, MAGIC);

	unsigned hb = rd16(HEARTBEAT);
	double beat = 0, last = 0, stat = 0;
	int job = 0, ready = 1;
	enum { TO_FILE, TO_NET, TO_USB } to = TO_FILE;
	while (!quit) {
		double t = now();
		if (t - beat >= 0.1) {
			beat = t;
			wr16(0, MAGIC);                                  /* another core may have had this memory */
			wr16(HEARTBEAT, ++hb);
		}
		if (use_usb && usb_fd < 0 && !job && t - usb_scan >= 2) { usb_scan = t; usb_find(); }
		if (usb_fd >= 0 && t - stat >= 0.25) {              /* the printer's own lines */
			stat = t;
			ready = usb_ready();
			wr16(STATUS, !ready);
		}
		if (usb_fd < 0 && !ready) { ready = 1; wr16(STATUS, 0); }

		unsigned n = (rd16(HEAD) - tail) & 0xffff;
		if (n > RING_N) {                                   /* not a ring of ours: start over */
			tail = rd16(HEAD);
			wr16(TAIL, tail);
			n = 0;
		}
		if (n) {
			uint8_t buf[RING_N];
			for (unsigned k = 0; k < n; k++) buf[k] = mb[RING + 2 * ((tail + k) % RING_N) + 1];
			if (!job) to = usb_fd >= 0 ? TO_USB : net_host[0] ? TO_NET : TO_FILE;
			int w = 0;
			switch (to) {
			case TO_USB: w = usb_fd >= 0 ? (ready ? usb_put(buf, n) : 0) : -1; break;
			case TO_NET: w = net_put(buf, n); break;
			default:     w = file_put(buf, n); break;
			}
			if (w < 0) {                                    /* the cable was pulled: the rest goes elsewhere */
				job = 0;
				continue;
			}
			if (w > 0) {
				job = 1;
				last = t;
				tail = (tail + w) & 0xffff;
				wr16(TAIL, tail);
				if (verbose > 1) fprintf(stderr, "falcon_lpt: %d bytes\n", w);
				continue;
			}
		} else if (job && t - last >= idle) {
			job = 0;
			file_end();
			net_end();
			if (verbose) fprintf(stderr, "falcon_lpt: job done\n");
		}
		usleep(n ? 20000 : 5000);
	}
	file_end();
	net_end();
	wr16(STATUS, 1);
	return 0;
}

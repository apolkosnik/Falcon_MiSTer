#include "Vtb_nvram_store.h"
#include "verilated.h"
#include "falcon_nvram.h"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

static std::unique_ptr<Vtb_nvram_store> top;
static std::string root;
static bool request;
static bool inject_write;
static unsigned checks;
static unsigned acknowledgements;
static void check(bool ok, const char *message)
{
    ++checks;
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
static void tick(unsigned n = 1)
{
    while (n--) {
        top->clk = 0; top->eval();
        top->clk = 1; top->eval();
        if (top->save_req) request = true;
    }
}
static void power_on(bool initialize = true)
{
    top.reset(new Vtb_nvram_store);
    request = false;
    top->cfg_vga = 1;
    top->reset = 1;
    if (initialize) { tick(200); top->reset = 0; tick(2); }
}
static void cpu_write(unsigned a, uint8_t value)
{
    check(!top->busy, "CPU is not running during NVRAM restore");
    top->bus_cs = top->bus_we = top->bus_stb = 1;
    top->bus_addr = 0; top->bus_din = a; tick();
    top->bus_stb = 0; tick();
    top->bus_addr = 1; top->bus_din = value; top->bus_stb = 1; tick();
    top->bus_cs = top->bus_we = top->bus_stb = 0; tick(2);
}
static uint8_t cpu_read(unsigned a)
{
    top->bus_cs = top->bus_we = top->bus_stb = 1;
    top->bus_addr = 0; top->bus_din = a; tick();
    top->bus_stb = top->bus_we = 0; tick(2);
    top->bus_addr = 1; top->bus_stb = 1; tick();
    uint8_t value = top->bus_dout;
    top->bus_cs = top->bus_stb = 0; tick();
    return value;
}
using Image = std::array<uint8_t, 50>;
static Image image()
{
    Image result;
    for (unsigned i = 0; i < result.size(); ++i) result[i] = cpu_read(i + 14);
    return result;
}
static std::string filename() { return root + "/saves/Falcon/falcon.nvram"; }
static Image file_image()
{
    Image result;
    std::ifstream f(filename(), std::ios::binary);
    f.read(reinterpret_cast<char *>(result.data()), result.size());
    check(f.gcount() == 50, "saved file contains 50 bytes");
    check(f.peek() == EOF, "saved file has no protocol trailer");
    return result;
}
static void wait_request()
{
    unsigned n = 10000;
    while (!request && n--) tick();
    check(request, "save request arrives");
}
static void settle()
{
    tick(2);
    unsigned n = 1000;
    while ((top->busy || !top->nv_ready) && n--) tick();
    check(n != 0, "restore/default initialization completes");
    tick(2);
}

// Transport adapters connect the actual Main implementation to the actual
// RTL endpoint. File operations below are the production implementation.
const char *getRootDir() { return root.c_str(); }
static unsigned selected_index;
static uint16_t word(uint16_t value)
{
    top->hps_din = value; top->io_strobe = 1; tick();
    top->io_strobe = 0; tick(3);
    return top->hps_dout;
}
static void begin_file(unsigned cmd)
{
    top->fp_enable = 1; tick(); word(cmd);
}
static void end_file() { top->fp_enable = 0; tick(4); }
void user_io_set_index(unsigned char index)
{
    selected_index = index;
    begin_file(0x55); word(index); end_file();
}
void user_io_set_download(unsigned char enable, int)
{
    begin_file(0x53); word(enable ? 0xFF : 0); end_file();
    if (!enable && selected_index == 4) ++acknowledgements;
}
void user_io_file_tx_data(const uint8_t *data, uint32_t size)
{
    begin_file(0x54);
    for (unsigned i = 0; i < size; ++i) word(data[i]);
    end_file();
}
void user_io_set_upload(unsigned char enable, int)
{
    begin_file(0x53); word(enable ? 0xAA : 0); end_file();
}
void user_io_file_rx_data(uint8_t *data, uint32_t size)
{
    begin_file(0x54);
    for (unsigned i = 0; i < size; ++i) {
        data[i] = word(0);
        if (inject_write && i == 25) { cpu_write(20, 7); inject_write = false; }
    }
    end_file();
}
uint16_t spi_uio_cmd(uint16_t cmd)
{
    check(cmd == 0x3C, "host polls the upload request command");
    top->io_enable = 1; tick(); uint16_t result = word(cmd);
    top->io_enable = 0; tick(4); request = false;
    return result;
}
static void download(unsigned index, const uint8_t *data, unsigned size)
{
    user_io_set_index(index); user_io_set_download(1, 0);
    user_io_file_tx_data(data, size); user_io_set_download(0, 0); settle();
}
static void checksum()
{
    unsigned sum = 0;
    for (unsigned i = 14; i < 62; ++i) sum += cpu_read(i);
    cpu_write(62, ~sum); cpu_write(63, sum);
}
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    char dir[] = "/tmp/falcon-nvram-test-XXXXXX";
    check(mkdtemp(dir) != nullptr, "create test storage"); root = dir;
    power_on();
    falcon_nvram_load(); settle();
    Image defaults = image();
    check(defaults[14] == 0 && defaults[15] == 0x1A, "VGA default image");
    wait_request(); falcon_nvram_poll();
    check(file_image() == defaults, "first boot saves its defaults");
    tick(6000); check(!request, "acknowledged unchanged image is not saved repeatedly");
    std::puts("PASS: missing file/default image/acknowledged save");

    cpu_write(20, 3); cpu_write(21, 2); cpu_write(24, 5); cpu_write(30, 0x82); checksum();
    Image custom = image();
    wait_request(); falcon_nvram_poll();
    check(file_image() == custom, "guest settings reach the file unchanged");
    power_on(false); // Restore races the first power-on default image builder.
    falcon_nvram_load(); settle(); top->reset = 0;
    check(image() == custom, "power cycle restores all 50 bytes after default initialization");
    tick(6000); check(!request, "restore does not mark the image dirty");
    std::puts("PASS: guest writes/save/power cycle/restore");

    cpu_write(21, 4); custom = image();
    top->reset = 1; tick(50); top->reset = 0;
    wait_request(); falcon_nvram_poll();
    check(image() == custom && file_image() == custom, "machine reset retains dirty data and save state");
    top->cfg_vga = 0; tick(100); check(image() == custom, "monitor input does not erase stored settings");
    std::puts("PASS: reset and monitor preservation");

    cpu_write(24, 9); custom = image(); wait_request();
    inject_write = true; falcon_nvram_poll();
    check(file_image() == custom, "upload is a stable snapshot during guest writes");
    wait_request(); falcon_nvram_poll();
    check(file_image() == image() && file_image()[6] == 7, "old acknowledgement preserves newer writes");
    std::puts("PASS: write during upload and stale acknowledgement");

    custom = image();
    std::string temp = filename() + ".tmp";
    check(mkdir(temp.c_str(), 0700) == 0, "inject a file creation failure");
    cpu_write(24, 11); wait_request(); unsigned ack = acknowledgements;
    falcon_nvram_poll();
    check(ack == acknowledgements, "failed save is not acknowledged");
    check(file_image() == custom, "failed save preserves the previous file");
    check(rmdir(temp.c_str()) == 0, "remove injected failure");
    wait_request(); falcon_nvram_poll();
    check(file_image() == image(), "failed save retries successfully");
    std::puts("PASS: file failure, atomic replacement and retry");

    custom = image();
    std::array<uint8_t, 51> bad{};
    download(3, bad.data(), 7);
    check(image() == custom, "truncated restore leaves NVRAM intact");
    download(3, bad.data(), 51);
    check(image() == custom, "oversize restore leaves NVRAM intact");
    download(9, bad.data(), 50);
    check(image() == custom, "unrelated downloads leave NVRAM intact");
    std::puts("PASS: malformed and unrelated downloads");

    // Force a guest write inside the 50-clock snapshot copy.
    cpu_write(24, 12); tick(265); cpu_write(21, 8);
    tick(70); check(!request, "write during capture cancels the torn snapshot");
    wait_request(); falcon_nvram_poll();
    check(file_image() == image(), "capture restarts after a quiet interval");
    std::puts("PASS: write during snapshot capture");

    cpu_write(24, 13); wait_request();
    download(4, bad.data(), 3);
    download(4, bad.data(), 4);
    falcon_nvram_poll();
    check(file_image() == image(), "malformed and wrong-generation acknowledgements do not clear dirty data");
    tick(6000); check(!request, "successful save settles after bad acknowledgements");
    cpu_write(0, 45); tick(6000); check(!request, "RTC clock writes do not save the NVRAM file");
    std::puts("PASS: bad acknowledgements and RTC-only writes");

    top->defaults = 1; tick(2); top->defaults = 0; settle();
    Image rgb = image();
    check(rgb[6] == 0 && rgb[7] == 0 && rgb[14] == 1 && rgb[15] == 0x2A,
          "explicit reset initializes defaults for the selected monitor");
    wait_request(); falcon_nvram_poll(); check(file_image() == rgb, "explicit defaults are persisted");
    std::puts("PASS: explicit default reset and save");
    std::printf("PASS: %u checks; production Main code + real hps_io/NVRAM/controller RTL\n", checks);
    top.reset();
    return 0;
}

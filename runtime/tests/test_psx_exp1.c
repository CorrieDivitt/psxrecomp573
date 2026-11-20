/* psxrecomp573: Expansion 1 device hook and 4 MiB main RAM, through the REAL
 * memory.c accessors (same link set as psx_ram_runtime_map_test).
 *
 *  1. With no device registered, Expansion 1 stays retail open bus.
 *  2. A registered device sees every load and store in 0x1F000000-0x1F7FFFFF
 *     from every segment (KUSEG/KSEG0/KSEG1), with the physical address and
 *     width, and supplies the loaded value.
 *  3. Addresses just outside the region never reach the device.
 *  4. 4 MiB geometry: the 8 MiB window folds onto 4 MiB (two mirrors), and
 *     only 2/4/8 MiB requests are accepted. */
#include "psx_memory.h"
#include "psx_exp1.h"
#include "mod_plugins.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern uint32_t psx_read_word(uint32_t addr);
extern uint16_t psx_read_half(uint32_t addr);
extern uint8_t  psx_read_byte(uint32_t addr);
extern void     psx_write_word(uint32_t addr, uint32_t val);
extern void     psx_write_half(uint32_t addr, uint16_t val);
extern void     psx_write_byte(uint32_t addr, uint8_t val);

static int failures;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static struct { uint32_t phys, value; int size, is_write; } last;
static int calls;

static uint32_t dev_read(void *u, uint32_t phys, int size) {
    (void)u;
    calls++; last.phys = phys; last.size = size; last.is_write = 0;
    return 0x12345678u ^ phys;          /* address-dependent, to catch mixups */
}
static void dev_write(void *u, uint32_t phys, uint32_t value, int size) {
    (void)u;
    calls++; last.phys = phys; last.value = value; last.size = size; last.is_write = 1;
}

static void test_open_bus(void) {
    psx_exp1_register(NULL);
    CHECK(psx_exp1_device_get() == NULL);
    CHECK(psx_read_word(0x9F400004u) == 0xFFFFFFFFu);
    CHECK(psx_read_half(0xBF400004u) == 0xFFFFu);
    CHECK(psx_read_byte(0x1F400004u) == 0xFFu);
    psx_write_word(0x1F500000u, 1);     /* ignored, must not crash */
}

static void test_device(void) {
    static const uint32_t segs[3] = { 0x00000000u, 0x80000000u, 0xA0000000u };
    psx_exp1_device d = { dev_read, dev_write, NULL };
    psx_exp1_register(&d);
    CHECK(psx_exp1_device_get() != NULL);
    for (int s = 0; s < 3; s++) {
        uint32_t a = segs[s] | 0x1F400004u;
        calls = 0;
        CHECK(psx_read_word(a) == (0x12345678u ^ 0x1F400004u));
        CHECK(calls == 1 && last.phys == 0x1F400004u && last.size == 4 && !last.is_write);
        CHECK(psx_read_half(a + 2) == (uint16_t)(0x12345678u ^ 0x1F400006u));
        CHECK(last.phys == 0x1F400006u && last.size == 2);
        CHECK(psx_read_byte(a + 1) == (uint8_t)(0x12345678u ^ 0x1F400005u));
        CHECK(last.phys == 0x1F400005u && last.size == 1);
        psx_write_word(segs[s] | 0x1F640090u, 0xA5A5u);
        CHECK(last.is_write && last.phys == 0x1F640090u && last.value == 0xA5A5u && last.size == 4);
        psx_write_half(segs[s] | 0x1F500000u, 0x0053u);
        CHECK(last.is_write && last.phys == 0x1F500000u && last.value == 0x0053u && last.size == 2);
        psx_write_byte(segs[s] | 0x1F600000u, 0x85u);
        CHECK(last.is_write && last.phys == 0x1F600000u && last.value == 0x85u && last.size == 1);
    }
    /* Region edges. */
    calls = 0;
    (void)psx_read_word(0x1F000000u);
    (void)psx_read_byte(0x1F7FFFFFu);
    CHECK(calls == 2);
    calls = 0;
    (void)psx_read_word(0x1F800000u);   /* scratchpad */
    (void)psx_read_word(0x1EFFFFFCu);   /* unmapped below the region */
    CHECK(calls == 0);
    psx_exp1_register(NULL);
}

static void test_ram_4mb(void) {
    CHECK(psx_set_main_ram_bytes(0x00300000u) == 0);
    CHECK(psx_set_main_ram_bytes(PSX_MAIN_RAM_BOARD_4MB_BYTES) == 1);
    psx_ram_apply_size_request();
    CHECK(memory_get_ram_bytes() == 0x00400000u);
    CHECK(!psx_ram_8mb_active());
    psx_write_word(0x803FFFF0u, 0xCAFEF00Du);
    CHECK(psx_read_word(0x803FFFF0u) == 0xCAFEF00Du);
    CHECK(psx_read_word(0x807FFFF0u) == 0xCAFEF00Du);     /* second mirror */
    psx_write_word(0x80200000u, 0x11111111u);
    CHECK(psx_read_word(0x80000000u) != 0x11111111u ||
          psx_read_word(0x80200000u) == 0x11111111u);    /* 2 MiB is not a mirror */
    psx_write_word(0x80000000u, 0x22222222u);
    CHECK(psx_read_word(0x80200000u) == 0x11111111u);
    CHECK(psx_ram_map_read(0x00600010u) == 0x00200010u);
    psx_ram_reset_size_request();
    psx_ram_apply_size_request();
    CHECK(memory_get_ram_bytes() == PSX_MAIN_RAM_RETAIL_BYTES);
}

/* Two symbols dma.c needs that psx_ram_runtime_map_stubs.c does not double. */
__attribute__((weak)) uint32_t psx_hr_gpu_hold_depth;
__attribute__((weak)) void psx_devices_service_to_now_cause(unsigned request) { (void)request; }

int main(void) {
    test_open_bus();
    test_device();
    test_ram_4mb();
    if (failures) { fprintf(stderr, "%d failure(s)\n", failures); return 1; }
    printf("psx_exp1_test: all checks passed\n");
    return 0;
}

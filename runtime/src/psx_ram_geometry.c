/* psx_ram_geometry.c -- live main-RAM geometry state (see psx_memory.h).
 *
 * Retail 2 MiB mirroring is the default and the only geometry reachable
 * without the opt-in 8 MB main RAM mod. A trusted activation plugin requests
 * expanded RAM (psx_mod_set_main_ram_8mb) before memory_init(), which applies
 * the request exactly once per boot (psx_ram_apply_size_request). Keeping the
 * state machine here, free of memory.c's device dependencies, lets the retail
 * guarantee be unit-tested against the same code the runtime links.
 *
 * The geometry is a size and its mask, nothing else: every consumer folds a
 * physical window address with `& g_psx_ram_mask`, so the retail path costs
 * one load of a hot global over the old compile-time constant. */
#include "psx_memory.h"
#include "mod_plugins.h"

#include <stdint.h>

uint32_t g_psx_ram_size = PSX_MAIN_RAM_RETAIL_BYTES;
uint32_t g_psx_ram_mask = PSX_MAIN_RAM_RETAIL_BYTES - 1u;

static uint32_t s_ram_bytes_requested;   /* 0 = retail */
static uint32_t s_ram_bytes_board;       /* 0 = retail; from the BIOS image */

uint32_t memory_get_ram_bytes(void) { return g_psx_ram_size; }
int psx_ram_8mb_active(void) {
    return g_psx_ram_size == PSX_MAIN_RAM_EXPANDED_BYTES;
}

void psx_ram_reset_size_request(void) {
    s_ram_bytes_requested = 0;
}

int psx_mod_set_main_ram_8mb(int enabled) {
    s_ram_bytes_requested = enabled ? PSX_MAIN_RAM_EXPANDED_BYTES : 0u;
    return 1;
}

/* psxrecomp573: boards with other DRAM sizes (Konami System 573: 4 MiB)
 * request their geometry the same way. Only 2, 4 and 8 MiB decode cleanly
 * in the 8 MiB window. */
int psx_set_main_ram_bytes(uint32_t bytes) {
    if (bytes != PSX_MAIN_RAM_RETAIL_BYTES && bytes != PSX_MAIN_RAM_BOARD_4MB_BYTES &&
        bytes != PSX_MAIN_RAM_EXPANDED_BYTES)
        return 0;
    s_ram_bytes_requested = bytes == PSX_MAIN_RAM_RETAIL_BYTES ? 0u : bytes;
    return 1;
}

/* The board's own DRAM, declared by the active BIOS image (PsxBiosImageInfo
 * main_ram_bytes). Applied by the next psx_ram_apply_size_request(); an
 * explicit request (psx_mod_set_main_ram_8mb / psx_set_main_ram_bytes) wins. */
int psx_ram_set_board_bytes(uint32_t bytes) {
    if (bytes == 0u || bytes == PSX_MAIN_RAM_RETAIL_BYTES) {
        s_ram_bytes_board = 0u;
        return 1;
    }
    if (bytes != PSX_MAIN_RAM_BOARD_4MB_BYTES && bytes != PSX_MAIN_RAM_EXPANDED_BYTES)
        return 0;
    s_ram_bytes_board = bytes;
    return 1;
}

void psx_ram_apply_size_request(void) {
    const uint32_t bytes = s_ram_bytes_requested ? s_ram_bytes_requested
                         : s_ram_bytes_board     ? s_ram_bytes_board
                                                 : PSX_MAIN_RAM_RETAIL_BYTES;
    g_psx_ram_size = bytes;
    g_psx_ram_mask = bytes - 1u;
}

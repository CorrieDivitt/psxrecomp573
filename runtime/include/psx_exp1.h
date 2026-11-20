#ifndef PSXRECOMP_PSX_EXP1_H
#define PSXRECOMP_PSX_EXP1_H

/* Expansion 1 device hook (psxrecomp573).
 *
 * On a retail PS1, physical 0x1F000000-0x1F7FFFFF (Expansion 1) has no device
 * and reads as open bus. Arcade boards built on the PS1 chipset, such as the
 * Konami System 573, put their own hardware there: flash banks, input ports,
 * an RTC, I/O boards and security cartridges. A host registers one device
 * that receives every guest access in the region. With no device registered
 * the region keeps its retail open-bus behaviour, so PS1 titles are unchanged.
 *
 * Accesses arrive with the physical address and the width in bytes (1, 2
 * or 4); reads return the value zero-extended to 32 bits. */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PSX_EXP1_BASE 0x1F000000u
#define PSX_EXP1_END  0x1F7FFFFFu

typedef struct psx_exp1_device {
    uint32_t (*read)(void *user, uint32_t phys, int size);
    void     (*write)(void *user, uint32_t phys, uint32_t value, int size);
    void     *user;
} psx_exp1_device;

/* Registers the device (copied), or clears it with NULL. Call before boot. */
void psx_exp1_register(const psx_exp1_device *device);
/* Current device, or NULL. */
const psx_exp1_device *psx_exp1_device_get(void);

#ifdef __cplusplus
}
#endif

#endif

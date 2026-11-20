/* psx_vram_geometry.h — the GPU's VRAM size, one place for every consumer.
 *
 * Retail PS1 GPUs have 1 MB of VRAM: 1024 x 512 16-bit pixels, Y wraps at
 * 512. The arcade CXD8561Q family on the Konami System 573 has 2 MB:
 * 1024 x 1024, and texture page bit 11 (GP0(E1h) and the polygon texpage
 * word) selects Y base +512 (MAME psxgpu "type 2": m_n_ty = (tpage & 0x10)
 * << 4 | (tpage & 0x800) >> 2; GPUSTAT.15 mirrors it). The 573 BIOS's GX700
 * CHECK tests both halves (chips 4L and 4P).
 *
 * A board fact fixed at build time: CMake option PSX_VRAM_2MB defines
 * PSX_VRAM_LINES=1024 for the whole runtime. Renderers that are not converted
 * refuse to start in a 2 MB build instead of drawing a 1 MB view of it.
 */
#ifndef PSX_VRAM_GEOMETRY_H
#define PSX_VRAM_GEOMETRY_H

#ifndef PSX_VRAM_LINES
#define PSX_VRAM_LINES 512
#endif

#if PSX_VRAM_LINES != 512 && PSX_VRAM_LINES != 1024
#error "PSX_VRAM_LINES must be 512 (1 MB) or 1024 (2 MB)"
#endif

#define PSX_VRAM_WIDTH   1024
#define PSX_VRAM_HEIGHT  PSX_VRAM_LINES
#define PSX_VRAM_YMASK   (PSX_VRAM_LINES - 1u)
#define PSX_VRAM_2MB     (PSX_VRAM_LINES == 1024)

/* Texpage bits the renderer consumes: X base 0-3, Y base 4, semi 5-6,
 * colours 7-8, plus Y base 2 (bit 11) on a 2 MB GPU. */
#if PSX_VRAM_2MB
#define PSX_TPAGE_MASK   0x9FFu
#else
#define PSX_TPAGE_MASK   0x1FFu
#endif

/* The same numbers as GLSL source text, for shader string literals. */
#if PSX_VRAM_2MB
#define PSX_VRAM_YMASK_S  "1023"
#define PSX_VRAM_HALF_H_S "512.0"
#define PSX_VRAM_H_S      "1024.0"
#else
#define PSX_VRAM_YMASK_S  "511"
#define PSX_VRAM_HALF_H_S "256.0"
#define PSX_VRAM_H_S      "512.0"
#endif

/* Texture page Y base in VRAM lines. */
#define PSX_TPAGE_Y(tpage) \
    ((((tpage) >> 4) & 1u) * 256u + (PSX_VRAM_2MB ? (((tpage) >> 11) & 1u) * 512u : 0u))

/* CLUT Y from the CLUT attribute (bits 6-15): 9 bits on 1 MB, 10 on 2 MB
 * (MAME: (clut >> 6) & 0x3ff). */
#define PSX_CLUT_Y(clut) (((clut) >> 6) & PSX_VRAM_YMASK)

#endif

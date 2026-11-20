# psxrecomp573: Konami System 573 support

This fork adds what a System 573 game needs from the PS1 runtime. The 573 hardware itself lives in the title repo ([Fighting-Mania-VR](https://github.com/CorrieDivitt/Fighting-Mania-VR), `runtime573/`); this fork only exposes the hooks. With none of them used, behaviour is unchanged for PS1 titles.

| Change | Files | Use |
| --- | --- | --- |
| Expansion 1 device hook | `runtime/include/psx_exp1.h`, `runtime/src/memory.c` | `psx_exp1_register()` routes every 8/16/32-bit guest load and store in physical `0x1F000000-0x1F7FFFFF` to a host device. Unregistered: retail open bus |
| 4 MiB main RAM | `runtime/include/psx_memory.h`, `runtime/src/psx_ram_geometry.c`, `runtime/src/boot_state.c` | `psx_set_main_ram_bytes(PSX_MAIN_RAM_BOARD_4MB_BYTES)` before `memory_init()`. The 8 MiB window folds onto 4 MiB; savestates carry a distinct "RBM4" cookie |
| SIO1 status | `runtime/include/sio.h`, `runtime/src/sio.c` | `sio1_set_board_status(0x0085)` makes `SIO1_STAT` report DSR plus TX ready, which the 573 security cartridge uses to say it is present. Default 0 (retail) |

Test: `psx_exp1_test` (runtime/tests/test_psx_exp1.c) covers open bus with no device, every width and segment through the real accessors, region edges, and the 4 MiB fold.

Still to do for 573 titles: the 700B01 BIOS profile below, and the SPU IRQ and SPU DMA completion paths the game's sound engine depends on (see the title repo's `docs/runtime-plan.md`).

## Boot path: Konami 700B01 BIOS (manifest, 2026-11-19)

The 573 boots its game from flash through Konami's own BIOS, `700b01.22g` (512 KiB, in MAME's `sys573.zip`). The plan is to recompile it with a new `bios/700B01.toml` profile, the same way SCPH-1001 and OpenBIOS are handled. The player supplies the image; it is not redistributable.

OpenBIOS was looked at first. Its `BOARD=system573` build compiles, but by its own README it does not boot games; that needs a custom shell. Konami's BIOS already contains that shell, so it is the faithful choice.

Byte-level facts. These came from capstone disassembly of the image in the cloud workspace and still need checking in Ghidra:

| What | Where | Evidence |
| --- | --- | --- |
| Reset | `0xBFC00000`: DEV2 setup, then `j 0xBFC00188` | disassembly |
| Bus setup table | ROM `0x140`: (address, value) pairs, applied until a zero address | loop at `0xBFC00188` |
| POST and RAM test | `0xBFC18000` (`j` from `0xBFC003C8`); 7-segment codes to `0x1F640010`; probes `0x1F40000E` bit 10 for board revision (`0xC80` or `0x4788` to `RAM_SIZE`) | disassembly |
| Kernel ramtext copy | ROM `0xBFC10000`, 2 × `0x4800` = `0x9000` bytes, to RAM `0xA0000500`. The watchdog is kicked between the two halves (`0xBFC004C0`, then `0xBFC003F0`), followed by `jr 0xA0000500` | disassembly; `jal` targets in `0x500-0xFFFF` cluster in ROM `0x10000-0x16FFF` |
| Program ROM | ROM `0x20000`: header word 0 is entry `0x9FC202A4`, word 1 is the DEV0 ctrl value `0x24173F47`, word 2 is watchdog `0x1F5C0000`, and +0x18 holds "Lisenced by Sony Computer Entertainment Inc.(SCEI)" | the kernel at `0xBFC050C8` checks the string at `0x1FC20018`, then `jalr` to word 0 |
| Program ROM code | runs in place from KSEG0 ROM (`0x9FC2xxxx`); `0x9FC20144` reads the PS-X EXE header at `0x9FC28000` | disassembly |
| EXE "GX700 CHECK" | ROM `0x28000`, text `0x80010000` size `0x10000`, pc `0x80012C64` | PS-X EXE header |
| EXE boot shell | ROM `0x60000`, text `0x80380000` size `0x18000`, pc `0x803847C0`. This is the flash and CD boot menu | header; strings; flash routines |
| Shell flash check | `0x803826EC(device)`: select device `0x8038D9AC`, page 0 `0x8038D920`, CRC at `0x1F000020`, header at `0x1F000024`, CRC over offsets 0,1,2,4,8… up to text size, stepping to the next 4 MiB page past `0x1F400000` | disassembly |
| Shell flash load | `0x80382A28(device)`: copies text from `0x1F000824` to `t_addr` in word runs, wrapping at each 4 MiB page | disassembly |
| Shell CD check | `0x803828CC` | disassembly |
| Kanji font | `0xBFC66000` and `0xBFC69D68` (Krom2RawAdd at ROM `0x160E0`) | disassembly |

Fighting Mania's onboard flash, as MAME holds it (`nvram/fghtmn/29f016a.31m` = even bytes, `27m` = odd bytes), passes that check:

- stored CRC `0x05DBCE61`, calculated `0x05DBCE61`
- pc `0x8002CCFC`, t_addr `0x80010000`, text `0x16B000`, sp `0x801FFFF0`

Unknowns, not to be guessed:

- the shell's device order (onboard, then card, then CD) and how it uses the DIP switches
- which code copies the `0x60000` EXE into RAM
- the exact end of the kernel ramtext. The copy is `0x9000` bytes, but the POST code at ROM `0x18000` is inside that window and only ever runs from ROM, so the window must not emit it twice (rule 12)
- the install slots the game's libapi patches

Next: import `700b01.22g` into Ghidra at `0xBFC00000`, with the two EXEs as RAM overlays. Generate seeds, write `bios/700B01.toml` with the copy windows above, then recompile and check the boot against MAME.

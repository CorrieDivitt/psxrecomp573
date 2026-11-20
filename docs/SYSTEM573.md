# psxrecomp573: Konami System 573 support

This fork adds what a System 573 game needs from the PS1 runtime. The 573 hardware itself lives in the title repo ([Fighting-Mania-VR](https://github.com/CorrieDivitt/Fighting-Mania-VR), `runtime573/`); this fork only exposes the hooks. With none of them used, behaviour is unchanged for PS1 titles.

| Change | Files | Use |
| --- | --- | --- |
| Expansion 1 device hook | `runtime/include/psx_exp1.h`, `runtime/src/memory.c` | `psx_exp1_register()` routes every 8/16/32-bit guest load and store in physical `0x1F000000-0x1F7FFFFF` to a host device. Unregistered: retail open bus |
| 4 MiB main RAM | `runtime/include/psx_memory.h`, `runtime/src/psx_ram_geometry.c`, `runtime/src/boot_state.c` | `psx_set_main_ram_bytes(PSX_MAIN_RAM_BOARD_4MB_BYTES)` before `memory_init()`. The 8 MiB window folds onto 4 MiB; savestates carry a distinct "RBM4" cookie |
| SIO1 status | `runtime/include/sio.h`, `runtime/src/sio.c` | `sio1_set_board_status(0x0085)` makes `SIO1_STAT` report DSR plus TX ready, which the 573 security cartridge uses to say it is present. Default 0 (retail) |

Test: `psx_exp1_test` (runtime/tests/test_psx_exp1.c) covers open bus with no device, every width and segment through the real accessors, region edges, and the 4 MiB fold.

Still to do for 573 titles: booting the flash executable without a disc, and the SPU IRQ and SPU DMA completion paths the game's sound engine depends on (see the title repo's `docs/runtime-plan.md`).

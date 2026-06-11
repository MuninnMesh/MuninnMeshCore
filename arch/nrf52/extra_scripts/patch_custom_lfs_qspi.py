"""
Patch CustomLFS QSPI chip table for MuziWorks external flash parts.

CustomLFS 0.2.1 only lists a few 16-Mbit QSPI parts. Muzi Base Duo uses a
Winbond W25Q128JVPQ (16 MB) and Base Uno a W25Q32JVSS (4 MB), so QSPI-backed
companion storage needs these JEDEC IDs to mount.

Behavior (registered for all nrf52_base envs):
- The table is patched at import time, BEFORE any compilation. The pre-ELF
  hook only VERIFIES: if the table was somehow still unpatched by link time,
  the objects were compiled from the unpatched source, so the build fails
  loudly with a "re-run" instruction instead of producing a firmware whose
  QSPI filesystem silently fails to mount.
- A missing insertion marker (e.g. a CustomLFS update reformatted the table)
  only fails Muzi envs; other nRF52 boards that don't need the patch get a
  warning and keep building.
"""

from pathlib import Path

Import("env")  # pylint: disable=undefined-variable


CHIP_ENTRIES = {
    "W25Q128JVPQ": '''  // Winbond W25Q128JVPQ - Muzi Base Duo
  {
    .jedec_id = {0xEF, 0x40, 0x18},
    .total_size = 16777216,             // 16MB
    .sector_size = 4096,
    .page_size = 256,
    .address_bits = 24,
    .read_opcode = 0xEB,
    .program_opcode = 0x32,
    .erase_opcode = 0x20,
    .status_opcode = 0x05,
    .supports_quad_read = true,
    .supports_quad_write = true,
    .quad_enable_register = 2,
    .quad_enable_bit = 1,
    .quad_enable_volatile = false,
    .max_clock_hz = 80000000,
    .write_timeout_ms = 5,
    .erase_timeout_ms = 400,
    .startup_delay_us = 10000,
    .name = "W25Q128JVPQ"
  },

''',
    "W25Q32JVSS": '''  // Winbond W25Q32JVSS - Muzi Base Uno
  {
    .jedec_id = {0xEF, 0x40, 0x16},
    .total_size = 4194304,              // 4MB
    .sector_size = 4096,
    .page_size = 256,
    .address_bits = 24,
    .read_opcode = 0xEB,
    .program_opcode = 0x32,
    .erase_opcode = 0x20,
    .status_opcode = 0x05,
    .supports_quad_read = true,
    .supports_quad_write = true,
    .quad_enable_register = 2,
    .quad_enable_bit = 1,
    .quad_enable_volatile = false,
    .max_clock_hz = 80000000,
    .write_timeout_ms = 5,
    .erase_timeout_ms = 400,
    .startup_delay_us = 10000,
    .name = "W25Q32JVSS"
  },

''',
}

MARKER = '''  // PUYA P25Q16H - Nordic boards
'''


def _is_muzi_env() -> bool:
    return "muzi" in env.subst("$PIOENV").lower() or "muzi" in env.subst("$BOARD").lower()


def _custom_lfs_qspi_source() -> Path:
    libdeps = Path(env.subst("$PROJECT_LIBDEPS_DIR"))
    pioenv = env.subst("$PIOENV")
    return libdeps / pioenv / "CustomLFS" / "src" / "CustomLFS_QSPIFlash.cpp"


def _missing_chips(content: str):
    return [name for name in CHIP_ENTRIES if name not in content]


def _patch_custom_lfs_qspi():
    source_path = _custom_lfs_qspi_source()
    if not source_path.exists():
        # No CustomLFS dependency for this env (or not installed yet); the
        # pre-ELF verifier below catches the installed-too-late case.
        print(f"CustomLFS QSPI patch: skipped, source not found at {source_path}")
        return

    content = source_path.read_text()
    missing = _missing_chips(content)
    if not missing:
        print("CustomLFS QSPI patch: OK - Muzi chips already supported")
        return

    if MARKER not in content:
        if _is_muzi_env():
            print("CustomLFS QSPI patch: FAILED - insertion marker not found "
                  "(CustomLFS table changed?); Muzi QSPI storage would not mount")
            env.Exit(1)
        else:
            print("CustomLFS QSPI patch: WARNING - insertion marker not found; "
                  "skipping (this board does not need the Muzi chip entries)")
        return

    addition = "".join(CHIP_ENTRIES[name] for name in missing)
    source_path.write_text(content.replace(MARKER, addition + MARKER))
    print(f"CustomLFS QSPI patch: OK - added {', '.join(missing)}")


def _verify_custom_lfs_qspi(target=None, source=None, env_arg=None, **kwargs):  # pylint: disable=unused-argument
    """Pre-ELF hook: objects are already compiled here, so patching now would
    lie. Verify instead and force a re-run if the table was patched too late."""
    if not _is_muzi_env():
        return
    source_path = _custom_lfs_qspi_source()
    if not source_path.exists():
        return
    if _missing_chips(source_path.read_text()):
        _patch_custom_lfs_qspi()  # patch the source for the NEXT build
        print("CustomLFS QSPI patch: FAILED - chip table was patched after "
              "compilation; re-run the build to compile the patched table")
        env.Exit(1)


env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf",
                 env.VerboseAction(_verify_custom_lfs_qspi, "Verifying CustomLFS QSPI patch..."))
_patch_custom_lfs_qspi()

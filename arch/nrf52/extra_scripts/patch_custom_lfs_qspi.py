"""
Patch CustomLFS QSPI chip table for Muzi Base Duo's W25Q128JVPQ flash.

CustomLFS 0.2.1 only lists a few 16-Mbit QSPI parts. Muzi Base Duo boards use
W25Q128JVPQ, so QSPI-backed companion storage needs this JEDEC ID to mount the
full 16 MB device.
"""

from pathlib import Path

Import("env")  # pylint: disable=undefined-variable


W25Q128_ENTRY = '''  // Winbond W25Q128JVPQ - Muzi Base Duo
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

'''


def _custom_lfs_qspi_source() -> Path:
    libdeps = Path(env.subst("$PROJECT_LIBDEPS_DIR"))
    pioenv = env.subst("$PIOENV")
    return libdeps / pioenv / "CustomLFS" / "src" / "CustomLFS_QSPIFlash.cpp"


def _patch_custom_lfs_qspi(target=None, source=None, env_arg=None, **kwargs):  # pylint: disable=unused-argument
    source_path = _custom_lfs_qspi_source()
    if not source_path.exists():
        print(f"CustomLFS QSPI patch: skipped, source not found at {source_path}")
        return

    content = source_path.read_text()
    if "W25Q128JVPQ" in content:
        print("CustomLFS QSPI patch: OK - W25Q128JVPQ already supported")
        return

    marker = '''  // PUYA P25Q16H - Nordic boards
'''
    if marker not in content:
        print("CustomLFS QSPI patch: FAILED - insertion marker not found")
        env.Exit(1)
        return

    source_path.write_text(content.replace(marker, W25Q128_ENTRY + marker))
    print("CustomLFS QSPI patch: OK - added W25Q128JVPQ 16MB support")


qspi_action = env.VerboseAction(_patch_custom_lfs_qspi, "Applying CustomLFS QSPI patch...")
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", qspi_action)
_patch_custom_lfs_qspi()

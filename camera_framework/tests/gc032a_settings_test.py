#!/usr/bin/env python3

import pathlib
import re


ROOT = pathlib.Path(__file__).resolve().parents[1]
DRIVER_DIR = ROOT / "core" / "driver" / "gc032a"
DRIVER_SOURCE = DRIVER_DIR / "gc032a.c"
TABLE_RE = re.compile(
    r"\{\s*(0x[0-9a-fA-F]+)\s*,\s*(0x[0-9a-fA-F]+)\s*\}"
)


def extract_function(source, start_marker, end_marker):
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


def read_table(name):
    path = DRIVER_DIR / name
    if not path.exists():
        raise AssertionError(f"missing {path}")
    pairs = [
        (int(register, 16), int(value, 16))
        for register, value in TABLE_RE.findall(path.read_text())
    ]
    if len(pairs) < 100 or pairs[-1] != (0xFE, 0x00):
        raise AssertionError(f"incomplete GC032A table: {name}")
    return pairs


def assert_serial_markers(pairs):
    page = 0
    marker_registers = {}
    for register, value in pairs:
        if register == 0xFE:
            page = value
        elif page == 3 and register in (0x60, 0x61, 0x62, 0x63):
            marker_registers[register] = value

    expected = {0x60: 0xCA, 0x61: 0x80, 0x62: 0x9D, 0x63: 0xB6}
    if marker_registers != expected:
        raise AssertionError(
            f"serial marker configuration mismatch: {marker_registers}"
        )


def main():
    serial_table = read_table("gc032a_serial_settings.h")
    read_table("gc032a_dvp_settings.h")
    assert_serial_markers(serial_table)

    driver_source = DRIVER_SOURCE.read_text()
    for text in ("gc032a_log_serial_config", "serial regs:"):
        if text in driver_source:
            raise AssertionError(f"bring-up diagnostics remain: {text}")

    apply_pixformat = extract_function(
        driver_source,
        "static int gc032a_apply_pixformat(pixformat_t pixformat)",
        "static int gc032a_apply_framesize",
    )
    if "gc032a_update_reg(GC032A_REG_OUTPUT_FORMAT" not in apply_pixformat:
        raise AssertionError("pixel format does not update P0:0x44")

    sensor_init = extract_function(
        driver_source,
        "static int gc032a_sensor_init(void)",
        "static int gc032a_open(void)",
    )
    table_write = sensor_init.index("gc032a_write_table")
    format_apply = sensor_init.index("gc032a_apply_pixformat")
    if table_write >= format_apply:
        raise AssertionError("RGB565 selection must follow the register table")

    print("gc032a_settings_test: PASS")


if __name__ == "__main__":
    main()

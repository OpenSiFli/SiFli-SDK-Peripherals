#!/usr/bin/env python3
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
DRIVER = ROOT / "core" / "driver" / "bf30a2" / "bf30a2.c"
HEADER = ROOT / "core" / "driver" / "bf30a2" / "bf30a2.h"
KCONFIG = ROOT / "Kconfig"
SCONSCRIPT = ROOT / "SConscript"
HANDLE = ROOT / "core" / "handle" / "camera_handle.h"


def read(path):
    return path.read_text(encoding="utf-8")


def test_bf30a2_uses_camera_handle_ops_only():
    source = read(DRIVER)
    assert "const camera_device_ops_t bf30a2_ops" in source
    assert "CAMERA_DRIVER_EXPORT(bf30a2, &bf30a2_ops)" in source
    assert ".capture_async = bf30a2_capture_async" in source
    assert ".start_stream = bf30a2_start_stream" in source
    assert "rt_device_register" not in source
    assert "MSH_CMD_EXPORT" not in source


def test_bf30a2_reports_real_capability():
    assert "FRAMESIZE_240X320" in read(HANDLE)
    source = read(DRIVER)
    assert "FRAMESIZE_240X320" in source
    assert ".max_buffer_size = BF30A2_FRAME_SIZE" in source
    assert "#define BF30A2_FRAME_SIZE" in read(HEADER)


def test_bf30a2_build_wiring_is_sensor_scoped():
    kconfig = read(KCONFIG)
    sconscript = read(SCONSCRIPT)
    assert "config SENSOR_USING_BF30A2" in kconfig
    # The driver has no menu options left: everything it needs is either a board
    # symbol or a constant in its own source.
    assert "SENSOR_USING_BF30A2" in sconscript
    assert "core/driver/bf30a2" in sconscript


def test_bf30a2_controls_are_board_owned():
    kconfig = read(KCONFIG)
    source = read(DRIVER)
    # Every pad belongs to the board now, including the sensor reset line: the
    # driver only consumes the pin index the board names.
    assert "CAMERA_BF30A2_RESET_PIN" in source
    assert "CAMERA_SCCB_SCL_PIN" not in kconfig
    assert "CAMERA_SCCB_SCL_PIN" not in source
    assert "HAL_PIN_Set" not in source
    assert "PAD_PA" not in source
    # The control bus comes from the board's own CAMERA_I2C_DEVICE_NAME.
    assert "SCCB_I2C_BUS_NAME" in source
    assert "camera_xclk_start(g_hw_config.xclk_frequency_hz)" in source


def test_bf30a2_names_its_bus_instead_of_muxing_it():
    """The SPI pads (including the CS pull the sensor needs) are the board's:
    the menu picks the SPI instance, the driver owns the device label."""
    kconfig = read(KCONFIG)
    source = read(DRIVER)
    # Selectable bus instance, but never a pad.
    assert "config CAMERA_BF30A2_SPI_BUS1" in kconfig
    assert "config CAMERA_BF30A2_SPI_BUS2" in kconfig
    assert "config CAMERA_BF30A2_SPI_BUS_NAME" in kconfig
    assert "CAMERA_BF30A2_SPI_BUS_NAME" in source
    assert "BF30A2_SPI_DEVICE_NAME" in source
    assert "SPI2_CS" not in source


def test_bf30a2_outputs_rgb565_msb_first():
    source = read(DRIVER)
    expected = """\
        *rgb++ = (uint8_t)(p0 >> 8);
        *rgb++ = (uint8_t)p0;
        *rgb++ = (uint8_t)(p1 >> 8);
        *rgb++ = (uint8_t)p1;"""
    assert expected in source


if __name__ == "__main__":
    test_bf30a2_uses_camera_handle_ops_only()
    test_bf30a2_reports_real_capability()
    test_bf30a2_build_wiring_is_sensor_scoped()
    test_bf30a2_controls_are_board_owned()
    test_bf30a2_names_its_bus_instead_of_muxing_it()
    test_bf30a2_outputs_rgb565_msb_first()

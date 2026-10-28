#!/usr/bin/env python3

import pathlib
import sys

import kconfiglib


ROOT = pathlib.Path(__file__).resolve().parents[1]
KCONFIG = ROOT / "Kconfig"
DRIVER_ROOT = ROOT / "core" / "driver"
# The framework Kconfig reads SoC symbols that only an SDK tree defines, so this
# test loads it through a thin SoC stub. The stub is not part of the repository:
# without it the test skips instead of failing, and the menu is verified with a
# real `scons --board=...` build.
SOC_ENTRY = pathlib.Path(__file__).resolve().parent / "Kconfig.soc_harness"

if not SOC_ENTRY.exists():
    print(f"camera_kconfig_test: SKIP (missing {SOC_ENTRY.name}; "
          f"the Kconfig SoC harness is not shipped with the framework)")
    sys.exit(0)


def load_config(sensor, interface=None):
    config = kconfiglib.Kconfig(str(SOC_ENTRY), warn=False)
    config.syms["PKG_USING_CAMERA_FRAMEWORK"].set_value(2)
    config.syms[sensor].set_value(2)
    if interface is not None:
        config.syms[interface].set_value(2)
    return config


def expect_value(config, symbol, expected):
    actual = config.syms[symbol].str_value
    if actual != expected:
        raise AssertionError(
            f"{symbol}: expected {expected}, got {actual}"
        )


def test_ov2640_selects_dvp():
    config = load_config("SENSOR_USING_OV2640")
    expect_value(config, "CAMERA_USING_DVP", "y")
    expect_value(config, "CAMERA_USING_SERIAL", "n")


def test_gc032a_dvp_selects_dvp():
    config = load_config(
        "SENSOR_USING_GC032A",
        "CAMERA_GC032A_INTERFACE_DVP_8BIT",
    )
    expect_value(config, "CAMERA_USING_DVP", "y")
    expect_value(config, "CAMERA_USING_SERIAL", "n")


def test_gc032a_serial_selects_only_serial():
    config = load_config(
        "SENSOR_USING_GC032A",
        "CAMERA_GC032A_INTERFACE_SERIAL_2BIT",
    )
    expect_value(config, "CAMERA_USING_DVP", "n")
    expect_value(config, "CAMERA_USING_SERIAL", "y")
    # The only serial capture path is the DCMI SPI packet mode: the GPIO
    # sampling + timer DMA backend is gone, together with its buffers.
    expect_value(config, "CAMERA_SERIAL_SPI_2BIT", "y")
    for symbol in (
        "CAMERA_SERIAL_GPIO_SAMPLING",
        "CAMERA_SERIAL_SAMPLE_BUFFER_SIZE",
        "CAMERA_SERIAL_WORKER_STACK_SIZE",
    ):
        if symbol in KCONFIG.read_text():
            raise AssertionError(f"GPIO serial sampling must stay removed: {symbol}")
    # The camera I2C pads come from the board's bsp_pinmux.c, and so does the
    # bus name: no SCCB option may come back into the framework menu.
    for symbol in (
        "CAMERA_SCCB_SCL_PIN",
        "CAMERA_SCCB_SDA_PIN",
        "CAMERA_SCCB_I2C_BUS_NAME",
        "CAMERA_SCCB_TIMEOUT_MS",
        "CAMERA_SCCB_MAX_HZ",
    ):
        if symbol in config.syms:
            raise AssertionError(f"SCCB option must live in board/driver code: {symbol}")
    # Board names, driver constants: the framework menu carries neither the
    # MCLK wiring nor any bus timing.
    for symbol in (
        "CAMERA_XCLK_PIN",
        "CAMERA_XCLK_ENABLE",
        "CAMERA_XCLK_PWM_DEVICE_NAME",
        "CAMERA_XCLK_PWM_CHANNEL",
        "CAMERA_XCLK_BOARD_MUXED",
        "CAMERA_READ_TIMEOUT_MS",
        "CAMERA_ARDUCAM_SPI_MOSI_PIN",
        "CAMERA_ARDUCAM_SPI_MISO_PIN",
        "CAMERA_ARDUCAM_SPI_SCLK_PIN",
        "CAMERA_ARDUCAM_SPI_CS_PIN",
        "CAMERA_ARDUCAM_SPI_DEVICE_NAME",
        "CAMERA_ARDUCAM_SPI_MAX_HZ",
        "CAMERA_ARDUCAM_FIFO_SCRATCH_SIZE",
        "CAMERA_ARDUCAM_FIFO_WORKER_STACK_SIZE",
        "CAMERA_BF30A2_SPI_DEVICE_NAME",
        "CAMERA_BF30A2_DMA_BUFFER_SIZE",
        "CAMERA_DCMI_PCLK_FALLING",
        "CAMERA_SERIAL_SPI_PACKET_SIZE",
        "CAMERA_SERIAL_SPI_PACKET_IDS",
        "CAMERA_SERIAL_SPI_SYNC_WORD",
        "CAMERA_SERIAL_SPI_SAMPLE_FALLING",
        "CAMERA_SERIAL_SPI_DI_SWAP",
    ):
        if f"config {symbol}" in KCONFIG.read_text():
            raise AssertionError(f"framework Kconfig must stay hardware-free: {symbol}")


def test_communication_form_follows_the_sensor():
    """The Data bus menu must show the transport of the compiled-in sensor:
    OV2640 / GC032A offer 8-bit DVP or a serial SPI link, and BF30A2 replaces
    the transport choice with the SPI instance it streams on."""
    ov2640 = load_config("SENSOR_USING_OV2640")
    if ov2640.syms["CAMERA_OV2640_INTERFACE_DVP"].choice is None:
        raise AssertionError("OV2640 communication form is not a choice")
    if ov2640.syms["CAMERA_OV2640_INTERFACE_DVP"].visibility == 0:
        raise AssertionError("OV2640 communication form is not user visible")
    if ov2640.syms["CAMERA_GC032A_INTERFACE_DVP_8BIT"].visibility != 0:
        raise AssertionError("inactive sensor's communication form must stay hidden")
    expect_value(ov2640, "CAMERA_OV2640_INTERFACE_ARDUCAM_FIFO", "n")

    gc032a = load_config("SENSOR_USING_GC032A", "CAMERA_GC032A_INTERFACE_SERIAL_2BIT")
    if gc032a.syms["CAMERA_GC032A_INTERFACE_SERIAL_2BIT"].choice is None:
        raise AssertionError("GC032A communication form is not a choice")
    if gc032a.syms["CAMERA_OV2640_INTERFACE_DVP"].visibility != 0:
        raise AssertionError("inactive sensor's communication form must stay hidden")

    bf30a2 = load_config("SENSOR_USING_BF30A2")
    expect_value(bf30a2, "CAMERA_BF30A2_SPI_BUS2", "y")
    expect_value(bf30a2, "CAMERA_BF30A2_SPI_BUS_NAME", "spi2")


def test_arducam_spi_bus_is_selectable():
    """The ArduCam module fixes SPI as transport but not the instance, so its
    bus choice appears only once the SPI form is picked."""
    dvp = load_config("SENSOR_USING_OV2640")
    if dvp.syms["CAMERA_ARDUCAM_SPI_BUS1"].visibility != 0:
        raise AssertionError("ArduCam SPI choice must stay hidden in DVP mode")

    arducam = load_config("SENSOR_USING_OV2640", "CAMERA_OV2640_INTERFACE_ARDUCAM_FIFO")
    if arducam.syms["CAMERA_ARDUCAM_SPI_BUS2"].choice is None:
        raise AssertionError("ArduCam SPI bus is not a choice")
    if arducam.syms["CAMERA_ARDUCAM_SPI_BUS2"].visibility == 0:
        raise AssertionError("ArduCam SPI bus choice must be visible")
    expect_value(arducam, "CAMERA_ARDUCAM_SPI_BUS_NAME", "spi2")
    expect_value(arducam, "CAMERA_DATA_BUS_ADAPTER_NAME", "arducam_fifo")


def test_gpio_dvp_backend_is_gone():
    """The GPIO/timer-DMA DVP simulation loses data and must stay removed."""
    kconfig = KCONFIG.read_text()
    if "CAMERA_DVP_BACKEND_GPIO" in kconfig:
        raise AssertionError("the GPIO DVP backend must not come back")
    for symbol in (
        "CAMERA_DVP_PCLK_PIN",
        "CAMERA_DVP_HSYNC_PIN",
        "CAMERA_DVP_VSYNC_PIN",
        "CAMERA_DVP_PINGPONG_POOL_SIZE",
    ):
        if symbol in kconfig:
            raise AssertionError(f"stale DVP pin option: {symbol}")

    data_bus = ROOT / "core" / "bus" / "data"
    for name in ("dvp.c", "dvp.h"):
        if (data_bus / name).exists():
            raise AssertionError(f"{name} must stay deleted")


def test_gpio_serial_sampling_is_gone():
    """The 2-bit serial GPIO sampling + timer DMA backend is not used on any
    board, so its sources, tests and buffer options must stay deleted."""
    data_bus = ROOT / "core" / "bus" / "data"
    for name in (
        "camera_serial.c",
        "camera_serial.h",
        "camera_serial_hw.c",
        "camera_serial_hw.h",
        "camera_serial_decoder.c",
        "camera_serial_decoder.h",
        "camera_serial_packer.c",
        "camera_serial_packer.h",
    ):
        if (data_bus / name).exists():
            raise AssertionError(f"{name} must stay deleted")

    sconscript = (ROOT / "SConscript").read_text()
    if "camera_serial" in sconscript:
        raise AssertionError("SConscript still builds the GPIO sampling backend")

    kconfig = KCONFIG.read_text()
    for symbol in (
        "CAMERA_SERIAL_GPIO_SAMPLING",
        "CAMERA_SERIAL_SAMPLE_BUFFER_SIZE",
        "CAMERA_SERIAL_WORKER_STACK_SIZE",
        "CAMERA_SERIAL_GPIO_PORT",
    ):
        if symbol in kconfig:
            raise AssertionError(f"GPIO serial sampling must stay removed: {symbol}")


def test_bf30a2_uses_24mhz_xclk():
    source = (DRIVER_ROOT / "bf30a2" / "bf30a2.c").read_text()
    if ".xclk_frequency_hz = 24000000U" not in source:
        raise AssertionError("BF30A2 must own its XCLK frequency")


def test_xclk_frequency_is_driver_owned():
    kconfig = KCONFIG.read_text()
    if "CAMERA_XCLK_FREQ" in kconfig:
        raise AssertionError("XCLK frequency must not be a global Kconfig option")

    expected = {
        "ov2640/ov2640.c": ".xclk_frequency_hz = 24000000U",
        "gc032a/gc032a.c": ".xclk_frequency_hz = 6000000U",
    }
    for relative_path, field in expected.items():
        source = (DRIVER_ROOT / relative_path).read_text()
        if field not in source:
            raise AssertionError(f"{relative_path}: missing driver-owned XCLK frequency")


def test_mclk_requirement_follows_the_sensor():
    """The sensor decides whether an MCLK is required; the board only says
    whether it supplies one. The two are separate derived symbols so a board
    that forgot its XCLK options fails to build instead of running clockless."""
    dvp = load_config("SENSOR_USING_OV2640")
    expect_value(dvp, "CAMERA_XCLK_REQUIRED", "y")
    expect_value(dvp, "CAMERA_BOARD_HAS_MCLK", "n")

    arducam = load_config("SENSOR_USING_OV2640", "CAMERA_OV2640_INTERFACE_ARDUCAM_FIFO")
    expect_value(arducam, "CAMERA_XCLK_REQUIRED", "n")

    expect_value(load_config("SENSOR_USING_GC032A"), "CAMERA_XCLK_REQUIRED", "y")
    expect_value(load_config("SENSOR_USING_BF30A2"), "CAMERA_XCLK_REQUIRED", "y")

    # Nothing declares CAMERA_XCLK_ENABLE in the harness, so no board MCLK: the
    # backend's #error is what keeps this from silently shipping.
    source = (ROOT / "core/bus/data/camera_xclk.c").read_text(encoding="utf-8")
    for token in ("CAMERA_XCLK_REQUIRED", "CAMERA_BOARD_HAS_MCLK",
                  "CAMERA_XCLK_EXTERNAL_OSC", "#error"):
        if token not in source:
            raise AssertionError(
                f"camera_xclk.c must gate the missing-MCLK case on {token}"
            )


def test_board_symbols_are_never_written_by_the_framework():
    """Dependency direction: the board declares, the framework reads.

    The framework may *test* a board symbol (a section may only activate when the
    board wired the hardware), but it must never *select* one: a select makes the
    board's value follow the framework, which is the inversion we want to avoid.
    The mirror rule - no CAMERA_FRAMEWORK_*/CAMERA_USING_* symbol inside
    customer/boards/ - lives in docs/hardware_ownership_refactor.md, because the
    board tree is not part of this repository.
    """
    for number, line in enumerate(KCONFIG.read_text().splitlines(), 1):
        stripped = line.strip()
        if stripped.startswith("select") and "BOARD_" in stripped:
            raise AssertionError(
                f"Kconfig:{number}: framework must not select a board symbol: "
                f"{stripped}"
            )


def test_serial_kconfig_does_not_select_spi():
    source = KCONFIG.read_text()
    serial_block = source.split("if CAMERA_USING_SERIAL", 1)[1].split(
        "endif", 1
    )[0]
    for symbol in (
        "BSP_USING_SPI2",
        "BSP_SPI2_RX_USING_DMA",
        "BSP_USING_SPI_DMA_CIRCULAR",
        "CAMERA_SERIAL_SPI_BUS_NAME",
    ):
        if symbol in serial_block:
            raise AssertionError(
                f"serial GPIO-DMA still selects SPI: {symbol}"
            )


if __name__ == "__main__":
    test_ov2640_selects_dvp()
    test_gc032a_dvp_selects_dvp()
    test_gc032a_serial_selects_only_serial()
    test_communication_form_follows_the_sensor()
    test_arducam_spi_bus_is_selectable()
    test_bf30a2_uses_24mhz_xclk()
    test_xclk_frequency_is_driver_owned()
    test_mclk_requirement_follows_the_sensor()
    test_board_symbols_are_never_written_by_the_framework()
    test_serial_kconfig_does_not_select_spi()
    test_gpio_dvp_backend_is_gone()
    test_gpio_serial_sampling_is_gone()
    print("camera_kconfig_test: PASS")

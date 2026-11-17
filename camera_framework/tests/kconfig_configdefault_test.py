#!/usr/bin/env python3
"""Board model/interface defaults use the named-choice stub supported by the SDK.

The board only reopens ``SENSOR_MODEL`` and the per-model interface choices to
set their defaults; the component owns the full choices (prompt + members). In a
component-less bootloader/ACPU pass each empty stub has an inactive dependency
and is inert, so no choice sanity warnings are produced.

Run with PYTHONPATH pointing at the SDK's ``tools/kconfig``.
"""

import atexit
import pathlib
import shutil
import sys

import kconfiglib


ROOT = pathlib.Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "Kconfig"
# Keep the scratch tree inside the repository: some sandboxes only allow the
# workspace to be written to. It is removed when the test exits.
WORK = ROOT.parent / ".kconfig_test_tmp"
WORK.mkdir(exist_ok=True)
atexit.register(shutil.rmtree, str(WORK), ignore_errors=True)


MODEL_SYMBOLS = {
    "ov2640": "SENSOR_USING_OV2640",
    "gc032a": "SENSOR_USING_GC032A",
    "bf30a2": "SENSOR_USING_BF30A2",
}


BOARD = """\
config BSP_USING_CAMERA1
    bool "Camera 1"
    default {camera}

if BSP_USING_CAMERA1 && PKG_USING_CAMERA_FRAMEWORK
    # Default-only stub: the component owns the full SENSOR_MODEL choice.
    choice SENSOR_MODEL
        default {default_model}
    endchoice

    # Default-only stub for the selected model's component-owned interface
    # choice (other models keep the component fallback).
{interface_stub}

    configdefault CAMERA_I2C_DEVICE_NAME
        default "i2c1"

    configdefault CAMERA_XCLK_ENABLE
        default y

    configdefault CAMERA_XCLK_PWM_DEVICE_NAME
        default "pwmt2"

    configdefault CAMERA_XCLK_PWM_CHANNEL
        default 1
endif
"""


INTERFACE_CHOICES = {
    "ov2640": {
        "dvp": ("CAMERA_OV2640_INTERFACE", "CAMERA_OV2640_INTERFACE_DVP"),
        "spi1": ("CAMERA_OV2640_INTERFACE", "CAMERA_OV2640_INTERFACE_SPI1"),
        "spi2": ("CAMERA_OV2640_INTERFACE", "CAMERA_OV2640_INTERFACE_SPI2"),
    },
    "gc032a": {
        "dvp": ("CAMERA_GC032A_INTERFACE", "CAMERA_GC032A_INTERFACE_DVP"),
        "serial2bit": ("CAMERA_GC032A_INTERFACE",
                       "CAMERA_GC032A_INTERFACE_SERIAL2BIT"),
    },
    "bf30a2": {
        "spi1": ("CAMERA_BF30A2_INTERFACE", "CAMERA_BF30A2_INTERFACE_SPI1"),
        "spi2": ("CAMERA_BF30A2_INTERFACE", "CAMERA_BF30A2_INTERFACE_SPI2"),
    },
}


def interface_stub(model, interface):
    """The board's single default-only stub for the selected model."""

    entry = INTERFACE_CHOICES[model].get(interface)
    if entry is None:
        return ""
    choice, default = entry
    return ("    choice {}\n"
            "        default {}\n"
            "    endchoice\n".format(choice, default))


def load(model="ov2640", interface="dvp", camera=True, with_component=True,
         extra_config=""):
    """Load a board stub plus the real component Kconfig."""

    board = WORK / "board.Kconfig"
    board.write_text(BOARD.format(
        default_model=MODEL_SYMBOLS[model],
        interface_stub=interface_stub(model, interface),
        camera="y" if camera else "n"))

    top = WORK / "top.Kconfig"
    sources = 'source "{}"\n'.format(board.as_posix())
    if with_component:
        sources += 'source "{}"\n'.format(COMPONENT.as_posix())
    top.write_text(sources)

    fragment = WORK / "in.config"
    fragment.write_text(
        "CONFIG_BSP_USING_CAMERA1={}\n".format("y" if camera else "n")
        + extra_config
    )

    config = kconfiglib.Kconfig(str(top), warn_to_stderr=False)
    config.load_config(str(fragment))
    # This is what tools/kconfig/kconfig.py calls before checking warnings.
    config.check_undefined_configdefaults()
    config.write_config(str(WORK / "out.config"))
    return config


def expect_value(config, symbol, expected):
    actual = config.syms[symbol].str_value
    if actual != expected:
        raise AssertionError("{}: expected {!r}, got {!r}"
                             .format(symbol, expected, actual))


def expect_no_warnings(config, case):
    if config.warnings:
        raise AssertionError("{} warned: {}"
                             .format(case, "; ".join(config.warnings)))


def expect_one(config, symbols, selected):
    for symbol in symbols:
        expect_value(config, symbol, "y" if symbol == selected else "n")


def check_model(model, selected):
    config = load(model=model)
    expect_no_warnings(config, model)
    expect_one(config, ("SENSOR_USING_OV2640", "SENSOR_USING_GC032A",
                        "SENSOR_USING_BF30A2"), selected)
    # The board default must still be reflected in the hardware symbols.
    expect_value(config, "CAMERA_I2C_DEVICE_NAME", "i2c1")
    expect_value(config, "CAMERA_XCLK_ENABLE", "y")
    expect_value(config, "CAMERA_XCLK_PWM_DEVICE_NAME", "pwmt2")
    expect_value(config, "CAMERA_XCLK_PWM_CHANNEL", "1")


def check_interface(model, interface, selected, backend):
    config = load(model=model, interface=interface)
    expect_no_warnings(config, "{} + {}".format(model, interface))
    prefixes = {
        "ov2640": ("CAMERA_OV2640_INTERFACE_DVP",
                   "CAMERA_OV2640_INTERFACE_SPI1",
                   "CAMERA_OV2640_INTERFACE_SPI2"),
        "gc032a": ("CAMERA_GC032A_INTERFACE_DVP",
                   "CAMERA_GC032A_INTERFACE_SERIAL2BIT"),
        "bf30a2": ("CAMERA_BF30A2_INTERFACE_SPI1",
                   "CAMERA_BF30A2_INTERFACE_SPI2"),
    }
    expect_one(config, prefixes[model], selected)
    expect_one(config, ("CAMERA_USING_DVP", "CAMERA_USING_SERIAL",
                        "CAMERA_USING_SPI_MODULE"), backend)
    if backend != "CAMERA_USING_SPI_MODULE":
        expect_value(config, "CAMERA_USING_ARDUCAM_FIFO", "n")


def test_board_default_selects_each_model():
    check_model("ov2640", "SENSOR_USING_OV2640")
    check_model("gc032a", "SENSOR_USING_GC032A")
    check_model("bf30a2", "SENSOR_USING_BF30A2")


def test_board_default_selects_each_interface():
    check_interface("ov2640", "dvp", "CAMERA_OV2640_INTERFACE_DVP",
                    "CAMERA_USING_DVP")
    check_interface("ov2640", "spi1", "CAMERA_OV2640_INTERFACE_SPI1",
                    "CAMERA_USING_SPI_MODULE")
    check_interface("ov2640", "spi2", "CAMERA_OV2640_INTERFACE_SPI2",
                    "CAMERA_USING_SPI_MODULE")
    check_interface("gc032a", "dvp", "CAMERA_GC032A_INTERFACE_DVP",
                    "CAMERA_USING_DVP")
    check_interface("gc032a", "serial2bit",
                    "CAMERA_GC032A_INTERFACE_SERIAL2BIT",
                    "CAMERA_USING_SERIAL")
    check_interface("bf30a2", "spi1", "CAMERA_BF30A2_INTERFACE_SPI1",
                    "CAMERA_USING_SPI_MODULE")
    check_interface("bf30a2", "spi2", "CAMERA_BF30A2_INTERFACE_SPI2",
                    "CAMERA_USING_SPI_MODULE")


def test_component_fallback_without_board_interface_stub():
    # No board stub: the component's plain fallback applies.
    for model, selected, backend in (
        ("ov2640", "CAMERA_OV2640_INTERFACE_DVP", "CAMERA_USING_DVP"),
        ("gc032a", "CAMERA_GC032A_INTERFACE_DVP", "CAMERA_USING_DVP"),
        ("bf30a2", "CAMERA_BF30A2_INTERFACE_SPI1", "CAMERA_USING_SPI_MODULE"),
    ):
        config = load(model=model, interface=None)
        expect_no_warnings(config, model + " fallback")
        expect_one(config, {
            "ov2640": ("CAMERA_OV2640_INTERFACE_DVP",
                       "CAMERA_OV2640_INTERFACE_SPI1",
                       "CAMERA_OV2640_INTERFACE_SPI2"),
            "gc032a": ("CAMERA_GC032A_INTERFACE_DVP",
                       "CAMERA_GC032A_INTERFACE_SERIAL2BIT"),
            "bf30a2": ("CAMERA_BF30A2_INTERFACE_SPI1",
                       "CAMERA_BF30A2_INTERFACE_SPI2"),
        }[model], selected)
        expect_one(config, ("CAMERA_USING_DVP", "CAMERA_USING_SERIAL",
                            "CAMERA_USING_SPI_MODULE"), backend)


def test_menuconfig_choices_win_over_board_defaults():
    config = load(model="ov2640", interface="dvp",
                  extra_config=("CONFIG_SENSOR_USING_GC032A=y\n"
                                "CONFIG_CAMERA_GC032A_INTERFACE_SERIAL2BIT=y\n"))
    expect_no_warnings(config, "menuconfig override")
    expect_value(config, "SENSOR_USING_GC032A", "y")
    expect_value(config, "SENSOR_USING_OV2640", "n")
    expect_value(config, "CAMERA_GC032A_INTERFACE_SERIAL2BIT", "y")
    expect_value(config, "CAMERA_GC032A_INTERFACE_DVP", "n")
    expect_value(config, "CAMERA_USING_SERIAL", "y")


def test_camera_off_disables_everything():
    config = load(model="gc032a", interface="serial2bit", camera=False)
    expect_no_warnings(config, "camera off")
    expect_value(config, "PKG_USING_CAMERA_FRAMEWORK", "n")
    for symbol in ("SENSOR_USING_OV2640", "SENSOR_USING_GC032A",
                   "SENSOR_USING_BF30A2",
                   "CAMERA_GC032A_INTERFACE_DVP",
                   "CAMERA_GC032A_INTERFACE_SERIAL2BIT"):
        expect_value(config, symbol, "n")


def test_component_absent_keeps_defaults_inert():
    """bootloader/ACPU (or the first sf-pkg pass) must not warn even when the
    connector is enabled: the PKG guard makes every configdefault inactive."""
    config = load(model="gc032a", interface="serial2bit",
                  with_component=False)
    expect_no_warnings(config, "component absent")


def test_component_owns_choices_without_string_mapping():
    component = COMPONENT.read_text()
    assert "choice SENSOR_MODEL" in component
    assert "choice CAMERA_OV2640_INTERFACE" in component
    assert "choice CAMERA_GC032A_INTERFACE" in component
    assert "choice CAMERA_BF30A2_INTERFACE" in component
    assert "CAMERA_SENSOR_MODEL" not in component
    assert "CAMERA_SENSOR_INTERFACE" not in component
    # The board only supplies default-only choice stubs; the component keeps
    # plain fallback defaults for every interface choice.
    for member in ("CAMERA_OV2640_INTERFACE_DVP",
                   "CAMERA_GC032A_INTERFACE_DVP",
                   "CAMERA_BF30A2_INTERFACE_SPI1"):
        assert "default {}".format(member) in component
    # The component must stay board-gated, not SoC-gated: BSP_USING_CAMERA1
    # decides whether the camera exists, so no SOC_SF32LB57X dependency may
    # hide the DVP/serial interfaces on another board that exposes DCMI.
    assert "SOC_SF32LB57X" not in component


if __name__ == "__main__":
    test_board_default_selects_each_model()
    test_board_default_selects_each_interface()
    test_component_fallback_without_board_interface_stub()
    test_menuconfig_choices_win_over_board_defaults()
    test_camera_off_disables_everything()
    test_component_absent_keeps_defaults_inert()
    test_component_owns_choices_without_string_mapping()
    print("kconfig_configdefault_test: PASS")
    sys.exit(0)

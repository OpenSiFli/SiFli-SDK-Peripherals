#!/usr/bin/env python3

import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]


def function_body(source, signature, next_signature):
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def check_sensor(path, open_signature, close_signature, next_signature):
    source = path.read_text(encoding="utf-8")
    open_body = function_body(source, open_signature, close_signature)
    close_body = function_body(source, close_signature, next_signature)

    if open_body.index("camera_xclk_start") > open_body.index("sccb_init"):
        raise AssertionError(f"{path.name}: XCLK must start before SCCB")
    if open_body.count("camera_xclk_stop") < 3:
        raise AssertionError(f"{path.name}: every open failure must stop XCLK")
    if "camera_xclk_stop" not in close_body:
        raise AssertionError(f"{path.name}: close must stop XCLK")


def test_sccb_leaves_pads_to_the_board():
    """The board muxes the camera I2C pads: the SCCB layer must not mux pins
    and must not carry pin options of its own."""
    source = (ROOT / "core/bus/control/sccb.c").read_text(encoding="utf-8")
    if "HAL_PIN_Set" in source:
        raise AssertionError("SCCB must not mux pins; the board owns them")

    kconfig = (ROOT / "Kconfig").read_text(encoding="utf-8")
    for symbol in ("CAMERA_SCCB_SCL_PIN", "CAMERA_SCCB_SDA_PIN"):
        if symbol in kconfig:
            raise AssertionError(f"SCCB pin option must not come back: {symbol}")


def test_mclk_is_driven_only_through_the_board_named_pwm():
    """MCLK is a board fact: bsp_pinmux.c muxes the pad and Kconfig.board names
    the PWM device and channel. The framework must program that PWM and nothing
    else - no pad lookup, no guessed device name, no silent fallback that would
    reprogram whichever timer channel happens to be muxed to a pad."""
    source = (ROOT / "core/bus/data/camera_xclk.c").read_text(encoding="utf-8")

    for text in ("HAL_PIN_", "PAD_PA", "PIN_PAD_", "GPTIM2_CH"):
        if text in source:
            raise AssertionError(
                f"camera_xclk.c must not touch pads or timer functions: {text}"
            )
    if '"pwmt2"' in source:
        raise AssertionError(
            "camera_xclk.c must not name a PWM device; the board names it"
        )
    if "CAMERA_XCLK_PWM_DEVICE_NAME" not in source:
        raise AssertionError("camera_xclk.c must use the board's PWM device name")
    if "#error" not in source:
        raise AssertionError(
            "a board that enables MCLK must get a compile error when it does not "
            "name the PWM device and channel"
        )


def main():
    check_sensor(
        ROOT / "core/driver/ov2640/ov2640.c",
        "static int sensor_open(void)\n{",
        "static int sensor_close(void)\n{",
        "static rt_size_t sensor_capture(",
    )
    check_sensor(
        ROOT / "core/driver/gc032a/gc032a.c",
        "static int gc032a_open(void)\n{",
        "static int gc032a_close(void)\n{",
        "static int gc032a_set_pixformat(",
    )

    test_sccb_leaves_pads_to_the_board()
    test_mclk_is_driven_only_through_the_board_named_pwm()

    print("camera_xclk_ownership_test: PASS")


if __name__ == "__main__":
    main()

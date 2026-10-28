# Camera Framework Tests

## Layout

- `common`: shared test harness and assertions
- `handle`: handle-layer tests with fake `camera_device_ops_t`
- `ov2640_jpeg_assembler_test.c`, `camera_xclk_test.c`: host-only unit tests

## Static Checks (run on the host, no board needed)

`PYTHONPATH` must point at the SDK's bundled `kconfiglib.py` (e.g.
`<SDK>/tools/build`).

```bash
cd camera_framework/tests
PYTHONPATH=<SDK>/tools/build python camera_xclk_ownership_test.py
PYTHONPATH=<SDK>/tools/build python camera_examples_capability_test.py
PYTHONPATH=<SDK>/tools/build python bf30a2_driver_review_test.py
PYTHONPATH=<SDK>/tools/build python gc032a_settings_test.py
PYTHONPATH=<SDK>/tools/build python camera_kconfig_test.py   # needs an SDK tree, see below
```

| Script | What it locks down |
| --- | --- |
| `camera_xclk_ownership_test.py` | SCCB muxes no pin; `camera_xclk.c` only drives the board-named PWM (no pad scan, no hard-coded device name, missing name is a compile error); XCLK starts before SCCB and is stopped on every open failure |
| `camera_examples_capability_test.py` | each example advertises a configuration its Kconfig/`proj.conf` can actually produce |
| `bf30a2_driver_review_test.py`, `gc032a_settings_test.py` | driver register-table / settings sanity |
| `camera_kconfig_test.py` | menu structure per sensor/interface, derived symbols, board-symbol direction. It loads the framework Kconfig through a small SoC stub; without that stub it prints `SKIP` and exits 0, so the Kconfig menu is best re-verified by a real `scons --board=...` build |
| `camera_sdcard_example_test.py` | the SD-card example keeps a single synchronous `take_photo` command (currently expects an RGB565 sensor in its `proj.conf`) |

## On-target Suite (`tests/handle`)

Builds the same framework sources as production code, but enables
`CAMERA_HANDLE_TESTING` and injects fake sensor ops through
`camera_handle_set_test_ops()`, so handle behavior is tested without a real
sensor driver or a real DVP backend.

Run after flashing:

```text
run_camera_handle_tests
run_camera_sensor_smoke
```

`run_camera_handle_tests` exercises the handle API with fake sensor operations.
`run_camera_sensor_smoke` opens the sensor selected in Kconfig and captures one
real frame. Camera pads are muxed by the board's `bsp_pinmux.c`, so the tests do
not configure any pin: board symbols (`CAMERA_I2C_DEVICE_NAME`,
`CAMERA_XCLK_*`) come from the board's `Kconfig.board`, and the direction is
board -> framework only (see `develop.md` sections 4.1, 4.3 and Appendix A).

## What The Handle Suite Covers

- init / deinit flow
- capture rejection while streaming
- streaming ready-frame queue behavior
- dropped-frame accounting
- idempotent `camera_stop_stream()` behavior

## Suggested Future Coverage

- `driver`: fake SCCB + fake data-bus adapters
- `bus`: board-level integration tests for concrete backends
- `examples`: smoke tests for command parsing and filesystem flows

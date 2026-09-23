# Camera Framework Tests

## Layout

- `common`: shared test harness and assertions
- `handle`: handle-layer tests with fake `camera_device_ops_t`

## Current Suite

The first runnable suite is `tests/handle`.

It builds the same camera framework sources as production code, but enables `CAMERA_HANDLE_TESTING` and injects fake sensor ops through `camera_handle_set_test_ops()`.

This lets the suite test handle-layer behavior without depending on a real OV2640 driver or a real DVP backend.

Run after flashing:

```text
run_camera_handle_tests
run_camera_sensor_smoke
```

`run_camera_handle_tests` exercises the handle API with fake sensor operations.
`run_camera_sensor_smoke` opens the sensor selected in Kconfig and captures one
real frame. For GC032A 2-bit SPI wiring, see
`examples/take_photo_to_sdcard/README_EN.md`; the handle and SD-card examples
use the same GPIO-DMA pins.

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

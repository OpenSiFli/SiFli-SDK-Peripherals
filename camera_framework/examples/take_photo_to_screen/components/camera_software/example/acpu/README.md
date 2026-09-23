<VSCode.Cell id="#VSC-gluereadme" language="markdown">
# Camera software ACPU glue

Project-local dual-core glue for the `take_photo_to_screen` example. The
software ISP and the JPEG codecs are owned by the camera framework
(`camera_framework/software/isp` and `camera_framework/software/jpeg`), so both
cores build those shared sources; this directory only carries the adapter.

## Contents

| path | role |
| --- | --- |
| `acpu/camera_sw_acpu_worker.c` | ACPU-side `acpu_main()` entry: dispatches the JPEG encode/decode, ISP and CPU-usage tasks |
| `hcpu/camera_sw_acpu_client.c` | HCPU-side synchronous client for the shared-buffer tasks |
| `hcpu/camera_sw_acpu_run_task.c` | Override of the SDK task helper (five-second synchronous timeout) |
| `include/` | Protocol DTOs and the client API |

## Build wiring

The HCPU project adds `hcpu/SConscript` whenever at least one software
component runs on the ACPU; the ACPU project adds `acpu/SConscript` and points
it at the framework ISP and JPEG build scripts. Both sides include the public
component headers from `CAMERA_ISP_ROOT/include` and `CAMERA_JPEG_ROOT/include`,
which the project `SConstruct` exports next to `CAMERA_SOFTWARE_ROOT`.

## Buffers and cache

The caller owns every input, output and destination buffer passed to
`camera_sw_acpu_*`. IPC pointers must refer to memory visible to both cores,
normally shared PSRAM. `acpu_run_task()` performs the cache clean/invalidate
before the request; callers must not reuse a buffer until the synchronous call
returns. The client never returns a pointer to static JPEG storage.

`camera_sw_acpu_encode_rgb565_be()` accepts the RGB565_BE input and a
caller-owned JPEG destination with its capacity;
`camera_sw_acpu_decode_jpeg()` and `camera_sw_acpu_process_rgb565_be()` operate
in place on caller-provided shared buffers.

## acpu_main and errors

The worker exports `acpu_main()` and must be linked into the ACPU image. It
handles the JPEG encode, JPEG decode, ISP and CPU-usage task IDs. All APIs
return `0` on success and `-1` on invalid geometry, insufficient capacity,
protocol mismatch, a missing selected codec, or an ACPU task failure. JPEG
decode supports a maximum source/output row width of 640 pixels and selects
decoder scaling to fit the destination capacity.

Standalone, hardware-free verification of the ISP, JPEG and dual-core
components lives in `camera_framework/examples/isp`,
`camera_framework/examples/jpeg` and `camera_framework/examples/acpu`.

</VSCode.Cell>
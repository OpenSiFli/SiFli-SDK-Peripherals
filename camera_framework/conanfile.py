from conan import ConanFile


class Camera_FrameworkRecipe(ConanFile):
    """SiFli camera framework source package.

    The recipe lives in the framework's own repository root, next to the
    sources it ships: `core/` (handle layer, sensor drivers, data buses and
    SCCB), `software/` (software ISP and software JPEG), plus the framework's
    `Kconfig` / `SConscript`. The directory layout is kept as-is, so the
    in-repo build and the packaged build compile exactly the same tree.

    The Kconfig root symbol is `PKG_USING_CAMERA_FRAMEWORK` (the SF-PKG
    convention); the framework's single-select sensor/bus options keep their
    original `SENSOR_USING_*` / `CAMERA_*` names.
    """

    name = "camera_framework"
    version = "0.0.1"

    license = "Apache-2.0"
    user = "sifli"
    author = "1jianai1"
    url = "https://github.com/OpenSiFli/SiFli-SDK-Peripherals"
    homepage = "https://github.com/OpenSiFli/SiFli-SDK-Peripherals"
    description = (
        "SiFli camera framework: camera_handle API, OV2640/GC032A/BF30A2 "
        "sensor drivers, DCMI DVP and 2-bit SPI backends, ArduCam SPI FIFO "
        "backend, SCCB control bus, software ISP and software JPEG."
    )
    topics = ("camera", "framework", "dvp", "dcmi", "ov2640", "gc032a", "bf30a2", "isp", "jpeg")

    support_sdk_version = "^2.4"

    # Sources are located in the same place as this recipe, copy them to the
    # recipe. Everything in the framework root is part of the package:
    # conanfile.py, Kconfig, SConscript, LICENSE, README.md/README_EN.md,
    # develop.md, core/, software/, examples/, script/ and tests/.
    # Only core/ + software/ are compiled; examples/ and tests/ are docs and
    # static checks and are never added to a firmware by this recipe.
    exports_sources = "*"

    python_requires = "sf-pkg-base/[^1.0]@sifli"
    python_requires_extend = "sf-pkg-base.SourceOnlyBase"

    def requirements(self):
        # Deliberately empty: the framework only depends on the SiFli SDK
        # (DCMI, I2C, PWM and SPI device drivers), which the consumer provides.
        pass

    def package_info(self):
        # Start from sf-pkg-base.SourceOnlyBase, which sets
        # libdirs = ["."] (the SConscript entry point the SDK build looks for)
        # and includedirs = ["include"].
        super().package_info()

        # This package keeps the framework's own layout instead of an
        # include/ tree, so point the consumer's CPPPATH at the public header
        # directories. The packaged SConscript adds the same paths to the
        # build environment; declaring them here also makes the generated
        # sf-pkgs/SConscript_conandeps correct on its own.
        self.cpp_info.includedirs = [
            "core",
            "core/handle",
            "core/driver",
            "core/bus/control",
            "core/bus/data",
        ]

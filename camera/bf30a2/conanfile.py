from conan import ConanFile

class Bf30A2_DriverRecipe(ConanFile):
    name = "bf30a2"
    version = "1.1.0"

    license = "Apache-2.0"
    user = "jingle_xie"
    author = "jingle_xie"
    url = "https://packages.sifli.com/zh/packages/jingle_xie/bf30a2"
    homepage = "https://packages.sifli.com/zh/packages/jingle_xie/bf30a2"
    description = "The BF30A2 SPI camera is equipped with a compliant device driver implementation."
    topics = ("driver", "camera", "bf30a2")

    support_sdk_version = "^2.4"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "*"

    python_requires = "sf-pkg-base/[^1.1.0]@sifli"
    python_requires_extend = "sf-pkg-base.SourceOnlyBase"

    def requirements(self):
        # add your package dependencies here, for example:
        # self.requires("fmt/8.1.1")
        pass

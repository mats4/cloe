# mypy: ignore-errors
# pylint: skip-file

import os
from pathlib import Path

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools import cmake, files, scm, env

required_conan_version = ">=1.52.0"


class CloeComponentFrustumCulling(ConanFile):
    name = "cloe-plugin-frustum-culling"
    url = "https://github.com/eclipse/cloe"
    description = "Cloe component plugin that changes coordinates of the sensed data and applies frustum culling"
    license = "Apache-2.0"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "pedantic": [True, False],
        "test": [True, False],
        "test_verbose": [True, False],
    }
    default_options = {
        "pedantic": True,
        "test": True,
        "test_verbose": False
    }
    generators = "CMakeDeps", "VirtualRunEnv"
    no_copy_source = True
    exports_sources = [
        "src/*",
        "CMakeLists.txt",
    ]

    def set_version(self):
        version_file = Path(self.recipe_folder) / "../../VERSION"
        if version_file.exists():
            self.version = files.load(self, version_file).strip()
        else:
            git = scm.Git(self, self.recipe_folder)
            self.version = git.run("describe --dirty=-dirty")[1:]

    def requirements(self):
        self.requires(f"cloe-runtime/{self.version}@cloe/develop")
        self.requires(f"cloe-models/{self.version}@cloe/develop")
        self.requires("fmt/9.1.0")
        if self.options.test:
            self.requires("gtest/1.13.0")     

    def layout(self):
        cmake.cmake_layout(self)

    def generate(self):
        tc = cmake.CMakeToolchain(self)
        tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.cache_variables["CLOE_PROJECT_VERSION"] = self.version
        tc.cache_variables["TargetLintingExtended"] = self.options.pedantic
        tc.generate()

    def build(self):
        cm = cmake.CMake(self)
        if self.should_configure:
            cm.configure()
        if self.should_build:
            cm.build()
        if self.should_test:
            if self.options.test_verbose:
                cm.test(cli_args=["ARGS=-V -j1"])
            else:
                cm.test(cli_args=["ARGS=-j1"])

    def package(self):
        cm = cmake.CMake(self)
        if self.should_install:
            cm.install()

    def package_id(self):
        self.info.requires["boost"].full_package_mode()
        del self.info.options.pedantic

    def package_info(self):
        self.cpp_info.set_property("cmake_find_mode", "both")
        self.cpp_info.set_property("cmake_file_name", self.name)
        self.cpp_info.set_property("pkg_config_name", self.name)

        if not self.in_local_cache: # editable mode
            libdir = os.path.join(self.build_folder, "lib");
            self.runenv_info.append_path("LD_LIBRARY_PATH", libdir)

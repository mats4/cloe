from pathlib import Path
from conans import CMake, ConanFile, RunEnvironment, tools


class CloeOak(ConanFile):
    name = "cloe-oak"
    license = "Apache-2.0"
    url = "https://github.com/eclipse/cloe"
    description = "Cloe web-server backend implementation"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "test": [True, False],
        "pedantic": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "test": True,
        "pedantic": True,
    }
    generators = "cmake"
    no_copy_source = True
    exports_sources = [
        "include/*",
        "src/*",
        "CMakeLists.txt",
    ]
    requires = [
        "cpp-netlib/0.13.0@cloe/stable",
    ]

    _cmake = None

    def set_version(self):
        version_file = Path(self.recipe_folder) / "../VERSION"
        if version_file.exists():
            self.version = tools.load(version_file).strip()
        else:
            git = tools.Git(folder=self.recipe_folder)
            self.version = git.run("describe --dirty=-dirty")[1:]

    def requirements(self):
        self.requires(f"cloe-runtime/{self.version}@cloe/develop")

    def build_requirements(self):
        if self.options.test:
            self.build_requires("gtest/[~1.10]")

    def _configure_cmake(self):
        if self._cmake:
            return self._cmake
        self._cmake = CMake(self)
        self._cmake.definitions["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        self._cmake.definitions["BuildTests"] = self.options.test
        self._cmake.definitions["TargetLintingExtended"] = self.options.pedantic
        self._cmake.configure()
        return self._cmake

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()
        if self.options.test:
            with tools.environment_append(RunEnvironment(self).vars):
                cmake.test()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        if self.in_local_cache:
            self.cpp_info.libs = tools.collect_libs(self)
        else:
            self.cpp_info.libs = ["cloe-oak"]

    def package_id(self):
        del self.info.options.test
        del self.info.options.pedantic

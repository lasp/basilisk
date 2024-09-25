from conan import ConanFile

class BasiliskConan(ConanFile):
  name = "Basilisk"
  version = "2.3.4"
  license = "ISC"
  languages = ["C", "C++"]
  generators = ["CMakeDeps", "CMakeToolchain"]

  settings = ["os", "compiler", "build_type", "arch"]
  requires = [
    "eigen/3.4.0",
    "protobuf/3.21.12",
    "cppzmq/4.5.0",
    "opencv/4.10.0",
  ]
  tool_requires = [
  ]

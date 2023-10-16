#!/bin/sh
set -x
#cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE="Release" -DCONAN_IN_LOCAL_CACHE="OFF" -DCONAN_COMPILER="gcc" -DCONAN_COMPILER_VERSION="11" -DCONAN_CXX_FLAGS="-m64" -DCONAN_SHARED_LINKER_FLAGS="-m64" -DCONAN_C_FLAGS="-m64" -DCMAKE_CXX_FLAGS="-m64" -DCMAKE_SHARED_LINKER_FLAGS="-m64" -DCMAKE_C_FLAGS="-m64" -DCONAN_LIBCXX="libstdc++11" -DCMAKE_INSTALL_PREFIX="/root/basilisk/package" -DCMAKE_INSTALL_BINDIR="bin" -DCMAKE_INSTALL_SBINDIR="bin" -DCMAKE_INSTALL_LIBEXECDIR="bin" -DCMAKE_INSTALL_LIBDIR="lib" -DCMAKE_INSTALL_INCLUDEDIR="include" -DCMAKE_INSTALL_OLDINCLUDEDIR="include" -DCMAKE_INSTALL_DATAROOTDIR="share" -DCMAKE_PREFIX_PATH="/root/basilisk/dist3/conan" -DCMAKE_MODULE_PATH="/root/basilisk/dist3/conan" -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY="ON" -DCONAN_EXPORTED="1" -DBUILD_OPNAV="True" -DBUILD_VIZINTERFACE="True" -DEXTERNAL_MODULES_PATH="" -DPYTHON_VERSION="4.0" -Wno-dev .

# git clone --branch 3.3.9 --single-branch https://gitlab.com/libeigen/eigen.git

mkdir -p build_rv32
cd build_rv32
pwd
cmake -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_TOOLCHAIN_FILE=../riscv.toolchain.cmake ..
cmake --build . --target SimulationDriver
file SimulationDriver

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

set(CMAKE_C_COMPILER /opt/riscv/bin/riscv32-unknown-elf-gcc)
set(CMAKE_CXX_COMPILER /opt/riscv/bin/riscv32-unknown-elf-g++)
set(CMAKE_ASM_COMPILER /opt/riscv/bin/riscv32-unknown-elf-gcc)

set(CMAKE_FIND_ROOT_PATH /opt/riscv)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

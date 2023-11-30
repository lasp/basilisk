set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

# Set gcc for rv32
set(CMAKE_C_COMPILER /opt/riscv_imfc/bin/riscv32-unknown-elf-gcc)
set(CMAKE_CXX_COMPILER /opt/riscv_imfc/bin/riscv32-unknown-elf-g++)
set(CMAKE_ASM_COMPILER /opt/riscv_imfc/bin/riscv32-unknown-elf-gcc)

# Make sure flags are completely reset.
set(CMAKE_CXX_FLAGS "")
set(CMAKE_C_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "")

# Compilation options
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Include debug magic
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-use-cxa-atexit")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi-tag -D_GLIBCXX_USE_CXX11_ABI=0")

# This forces definition of __dso_handle in main-mrp-steering-controllers.cpp, but only when using
# this toolchain file
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMY_RV32=1")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMY_RV32=1")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -DMY_RV32=1")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-unwind-tables -fno-asynchronous-unwind-tables")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-unwind-tables -fno-asynchronous-unwind-tables")

# Linker options
# Remove unused sections, and print the sections:
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ffunction-sections -fdata-sections -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-warn-rwx-segments")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lm")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -nostartfiles")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -nostdlib")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-use-cxa-atexit")

# Options for MicroChip IMFC
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=rv32imfc_zicsr -mabi=ilp32")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=rv32imfc_zicsr -mabi=ilp32")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T /share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv-rv32-ram.ld")

# Ozone happiness
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -gstrict-dwarf -gdwarf-4 -fdebug-prefix-map=/share=/Users/dinkelk/projects/ema")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gstrict-dwarf -gdwarf-4 -fdebug-prefix-map=/share=/Users/dinkelk/projects/ema")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -gstrict-dwarf -gdwarf-4 -fdebug-prefix-map=/share=/Users/dinkelk/projects/ema")

# Set assembly flags to be the same as the C flags:
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

set(CMAKE_FIND_ROOT_PATH /opt/riscv_imfc)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(RISCV_COMPILE TRUE)

set(PLATFORM_SRC
      # This is for microchip
      "/share/ema-fsw-simple-benchmark/RV32imc/benchPlatform.c"
      "/share/ema-fsw-simple-benchmark/RV32imc/fpga_design_config/fpga_design_config.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_hal.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_hal.c"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_init.c"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_hal_version.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_assert.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_plic.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_regs.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_stubs.c"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_subsys.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_syscall.c"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/sample_fpga_design_config.h"
      "/share/ema-fsw-simple-benchmark/RV32imc/platform/miv_rv32_hal/miv_rv32_entry.S"
)

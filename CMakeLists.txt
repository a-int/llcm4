cmake_minimum_required(VERSION 3.26.3)
project(stm_manual CXX  C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(
    -DSTM32F411xE
)

set(arch_flags 
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
    -mabi=aapcs
)

# Compiler and Linker options
set(common_flags
    $<$<C_COMPILER_ID:GNU>:--specs=nosys.specs>
    $<$<C_COMPILER_ID:GNU>:--specs=nano.specs>
)

set(compile_flags
    -Wall -Wextra
    -Wshadow
    -Wdouble-promotion
    -Wformat=2 
    -Wformat-truncation
    -Wformat-signedness
    -Wundef
    -Wconversion
    # -Werror
    $<$<COMPILE_LANGUAGE:CXX>:-Weffc++>
    -ffunction-sections 
    -fdata-sections 
    -fstack-usage 
    -fno-strict-aliasing 
    -fno-builtin 
    -ffast-math
    -fno-common
    -fstack-usage -Wstack-usage=256
    $<$<COMPILE_LANGUAGE:C>:-std=c11>
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    -g3
)

# Linker Options
set(link_flags
    -Wl,--gc-sections -static
    -T ${CMAKE_CURRENT_LIST_DIR}/include/vendor/STM32F411CEUX.ld
    -flto
    -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
    -Wl,--print-memory-usage
    -Xlinker -Map=output.map 
)

add_compile_options(
    ${arch_flags}
    ${common_flags}
    ${compile_flags}
)

# include after options have been defined
add_subdirectory("src/")

# add the executable
# all code comes from libraries -> need to have a dummy empty file

if (NOT DEFINED BUILD_EXAMPLES)
  set(BUILD_EXAMPLES OFF)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory("example/")
endif()

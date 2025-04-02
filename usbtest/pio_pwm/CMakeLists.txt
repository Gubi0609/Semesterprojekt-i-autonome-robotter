cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk fra den installerede placering
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico CACHE STRING "Board type")

# Importer Pico SDK
include(pico_sdk_import.cmake)

# Projektdeklaration (understøtter C, C++ og ASM)
project(pio_pwm C CXX ASM)

# Initialiser Pico SDK
pico_sdk_init()

# Opret eksekverbar – tilføj både din C++-fil (main.cpp) og evt. C-kilde (pwm.c)
add_executable(pio_pwm
    main.cpp
    pwm.c
)

# Generer header fra PIO-programmet
pico_generate_pio_header(pio_pwm ${CMAKE_CURRENT_LIST_DIR}/pwm.pio)

# Link mod nødvendige biblioteker: pico_stdlib og hardware_pio (inkluderer f.eks. PWM)
target_link_libraries(pio_pwm PRIVATE pico_stdlib hardware_pio)

# Generer ekstra output-filer (bin, uf2 osv.) til flashing af Pico'en
pico_add_extra_outputs(pio_pwm)

# evt. tilføj en URL via pico_set_program_url hvis ønsket

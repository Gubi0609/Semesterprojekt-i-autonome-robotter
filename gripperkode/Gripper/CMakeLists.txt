# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Angiv standardsprogversioner for C og C++
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Angiv den ønskede Pico-boardtype (her: pico_w)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Inkludér Pico SDK-importfilen
include(pico_sdk_import.cmake)

# Definér projektet – vælg et entydigt navn
project(gripper C CXX ASM)

# Initialiser Pico SDK
pico_sdk_init()

# Tilføj eksekverbar fil – hvis du bruger C++ skal filen hedde main.cpp,
# eller main.c hvis du arbejder i C.
add_executable(gripper main.cpp)

# Link til nødvendige biblioteker: pico_stdlib og hardware_pwm (for PWM-funktionerne)
target_link_libraries(gripper
    pico_stdlib
    hardware_pwm
    hardware_adc
)

# Konfigurer standard I/O: Vælg om du vil bruge UART eller USB
pico_enable_stdio_uart(gripper 0)
pico_enable_stdio_usb(gripper 1)

# Generer ekstra output-filer (fx .uf2) til flashing af Pico'en
pico_add_extra_outputs(gripper)

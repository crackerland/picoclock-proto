include_directories(./include ../../rpi-base/src/peripherals/include)
add_library(screen ./src/LcdScreen.c)
target_link_libraries(screen PUBLIC core pico_stdlib hardware_spi hardware_i2c hardware_pwm hardware_adc)

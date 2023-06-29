include_directories(./include ../../rpi-base/src/peripherals/include ../../rpi-base/src/graphics/include)
add_library(gc9a01a ./src/LcdScreen.c)

target_link_libraries(gc9a01a PUBLIC core pico_stdlib hardware_spi hardware_i2c hardware_pwm hardware_adc)

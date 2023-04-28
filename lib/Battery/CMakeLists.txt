include_directories(./include)
add_library(battery ./src/PicoBattery.c)
target_link_libraries(battery PUBLIC pico_stdlib hardware_spi hardware_i2c hardware_pwm hardware_adc)

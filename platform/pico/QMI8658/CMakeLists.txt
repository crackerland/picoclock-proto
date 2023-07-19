aux_source_directory(. DIR_QMI8658_SRCS)
include_directories(../Config)
add_library(QMI8658 ${DIR_QMI8658_SRCS})
target_link_libraries(QMI8658 PUBLIC pico_stdlib hardware_i2c hardware_adc)

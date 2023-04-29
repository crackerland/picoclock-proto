include_directories(./include)
add_library(datetime ./src/DateTimeFormatter.c)
target_link_libraries(datetime PUBLIC pico_stdlib)

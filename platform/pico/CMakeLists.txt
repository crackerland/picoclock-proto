cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
set(PICO_BOARD="waveshare_rp2040_lcd_1.28")
pico_sdk_init()

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(RPI_LIB ${ROOT}/rpi-base/lib)
set(RPI_CORE ${ROOT}/rpi-base/src/core)
set(RPI_PERIPHERALS ${ROOT}/rpi-base/src/peripherals)
set(RPI_FONT ${ROOT}/rpi-base/src/font)
set(RPI_GRAPHICS ${ROOT}/rpi-base/src/graphics)
set(RPI_IO ${ROOT}/rpi-base/src/io)
set(RPI_ZLIB ${ROOT}/rpi-base/src/zlib)
set(RPI_CHRONO ${ROOT}/rpi-base/src/chrono)
add_library(core 
  ${RPI_CORE}/src/Arrays.c
	${RPI_CORE}/src/MathHelper.c
	${RPI_CORE}/src/ByteHelper.c
	${RPI_CORE}/src/crc.c
	${RPI_CORE}/src/collections/LinkedList.c
	${RPI_CORE}/src/collections/IterableQueryable.c
	${RPI_CORE}/src/print_helper.c
	${RPI_CORE}/src/Tasks.c)

add_library(font
  ${RPI_FONT}/src/FontPainterEx.c
  ${RPI_FONT}/src/FramebufferFontPainter.c)

add_library(graphics
	${RPI_GRAPHICS}/src/Color.c
  ${RPI_GRAPHICS}/src/BufferTexture.c
  ${RPI_GRAPHICS}/src/Texture16.c
  ${RPI_GRAPHICS}/src/SystemFontTextView.c
  ${RPI_GRAPHICS}/src/BufferTextureRenderer.c
  ${RPI_GRAPHICS}/src/ScreenTextureRenderer.c
  ${RPI_GRAPHICS}/src/TextureFontPainter.c)
  # ${RPI_GRAPHICS}/src/StreamBitmap.c
	# ${RPI_GRAPHICS}/src/StreamPng.c

add_library(io
  ${RPI_IO}/src/MemoryStream.c)

add_library(chrono 
  ${RPI_CHRONO}/src/DefaultDateTimeFormatter.c)

# add_library(zlib
#   ${RPI_ZLIB}/src/adler32.c 
# 	${RPI_ZLIB}/src/compress.c 
# 	${RPI_ZLIB}/src/crc32.c 
# 	${RPI_ZLIB}/src/deflate.c 
# 	${RPI_ZLIB}/src/gzclose.c 
#   ${RPI_ZLIB}/src/gzlib.c 
#   ${RPI_ZLIB}/src/gzread.c 
#   ${RPI_ZLIB}/src/gzwrite.c 
#   ${RPI_ZLIB}/src/infback.c 
#   ${RPI_ZLIB}/src/inffast.c 
#   ${RPI_ZLIB}/src/inflate.c 
#   ${RPI_ZLIB}/src/inftrees.c 
#   ${RPI_ZLIB}/src/trees.c 
#   ${RPI_ZLIB}/src/uncompr.c 
#   ${RPI_ZLIB}/src/zutil.c)

include_directories(./include)
include_directories(${RPI_CORE}/include)
include_directories(${RPI_FONT}/include)
include_directories(${RPI_PERIPHERALS}/include)
include_directories(${RPI_GRAPHICS}/include)
include_directories(${RPI_IO}/include)
# include_directories(${RPI_ZLIB}/include)
include_directories(${RPI_CHRONO}/include)

add_subdirectory(${ROOT}/lib/lcd ./bin/lcd)
add_subdirectory(${ROOT}/lib/QMI8658 ./bin/QMI8658)
add_subdirectory(${ROOT}/lib/GC9A01A ./bin/GC9A01A)
include_directories(
  ${ROOT}/include
  ${ROOT}/lib/lcd
  ${ROOT}/lib/GC9A01A/include
  ${ROOT}/lib/QMI8658)

add_executable(main
  main.c
  ${ROOT}/src/PicoClock.c
  ${ROOT}/src/Painter.c
  ./src/ScreenFramebuffer.c
  ./src/PicoDateTimeProvider.c
  ./src/PicoPowerManager.c
  ./src/PicoBattery.c
  ./src/PicoTimer.c
  ./src/PicoColorConverter.c)

set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 1000)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)

target_link_libraries(main
lcd QMI8658
pico_stdlib
pico_multicore
hardware_spi
hardware_i2c
hardware_pwm
hardware_rtc
hardware_timer
hardware_adc
hardware_flash
hardware_sync
pico_bootsel_via_double_reset
core
graphics
font
gc9a01a
io
lcd
chrono
)

add_custom_command(
  TARGET main POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_CURRENT_BINARY_DIR}/main.uf2
          ${CMAKE_CURRENT_BINARY_DIR}/../uf2/main.uf2
)

export CC = arm-none-eabi-gcc
export AS = arm-none-eabi-gcc
export LD = arm-none-eabi-gcc
export AR = arm-none-eabi-ar

export OPTIMIZATION = -O0
export LIB = $(abspath lib)
export BIN = $(abspath bin)
export ASFLAGS = -O0 -nostartfiles -mfloat-abi=hard -mfpu=vfp -march=armv6zk -mtune=arm1176jzf-s 
ROOT = $(abspath ../..)
RPI_ROOT = ${ROOT}/rpi-base
ROOT_SRC = ${RPI_ROOT}/src
PI_PLATFORM = ${RPI_ROOT}/platform/pi

#  -Werror=format
CFLAGS = -std=c99 \
	${ASFLAGS} -Wl,-T,${PI_PLATFORM}/runtime/rpi.x \
	-Wall -Wextra \
    -Werror=int-conversion -Werror=return-type -Werror=format-extra-args -Werror=implicit-function-declaration -Werror=overflow -Werror=unused-value -Werror=incompatible-pointer-types \
    -Wno-unused-function -Wno-unused-parameter \
    -fdiagnostics-color=always \
	-I${PI_PLATFORM}/runtime/include -I${PI_PLATFORM}/bcm2835/include \
	-I${ROOT_SRC}/core/include -I${ROOT_SRC}/graphics/include -I${ROOT_SRC}/filesystem/include -I${ROOT_SRC}/peripherals/include \
	-I${ROOT_SRC}/font/include -I${ROOT_SRC}/io/include \
	-DRPI_ZERO 

LDFLAGS = -static -L${LIB}
LDLIBS = -lpi -lbcm2835 -lgraphics -lfilesystem -lcore -lfont -lm

OBJS = main.o

MAIN = ${BIN}/kernel

$(MAIN): $(OBJS) elf img 

$(OBJS): depend

depend:
	$(MAKE) -C ${PI_PLATFORM} \
	&& $(MAKE) -C ${ROOT_SRC}/core \
	&& $(MAKE) -C ${ROOT_SRC}/io \
	&& $(MAKE) -C ${ROOT_SRC}/filesystem \
	&& $(MAKE) -C ${ROOT_SRC}/graphics \
	&& $(MAKE) -C ${ROOT_SRC}/font

elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LDLIBS) -o ${BIN}/kernel.elf

# Strips the extra data from the elf and converts it to the img.
img: elf
	arm-none-eabi-objcopy kernel.elf -O binary ${BIN}/kernel.img

.PHONY: clean clean-all

clean-all: clean 
	$(MAKE) -C ${PI_PLATFORM} clean \
	&& $(MAKE) -C ${ROOT_SRC}/core clean \
	&& $(MAKE) -C ${ROOT_SRC}/filesystem clean \
	&& $(MAKE) -C ${ROOT_SRC}/font clean \
	&& $(MAKE) -C ${ROOT_SRC}/graphics clean

clean:
	$(RM) $(OBJS) $(MAIN) kernel.elf ${BIN}/kernel.img
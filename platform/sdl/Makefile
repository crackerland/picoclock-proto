export CC = gcc
export AS = gcc
export LD = ld
export AR = ar

export LIB = $(abspath lib)
export OPTIMIZATION = -O0 -g -DDEBUG
BIN = $(abspath bin)
ROOT = $(abspath ../..)
RPI_ROOT = ${ROOT}/rpi-base
ROOT_SRC = ${RPI_ROOT}/src
SDL_PLATFORM = ${RPI_ROOT}/platform/sdl
SDL = ${SDL_PLATFORM}/SDL2-2.24.0/x86_64-w64-mingw32
SDL_IMG = ${SDL_PLATFORM}/SDL2_image-2.6.2/x86_64-w64-mingw32

CFLAGS = -std=c99 $(OPTIMIZATION) \
	-Wall -Wextra \
    -Werror=sign-compare -Werror=int-conversion -Werror=return-type -Werror=format -Werror=format-extra-args -Werror=implicit-function-declaration -Werror=overflow -Werror=unused-value -Werror=incompatible-pointer-types \
    -Wno-unused-function -Wno-unused-parameter -Wno-switch \
    -fdiagnostics-color=always \
	-I${SDL}/include/SDL2 -I${SDL_IMG}/include/SDL2 \
	-I${ROOT_SRC}/core/include -I${ROOT_SRC}/graphics/include -I${ROOT_SRC}/filesystem/include -I${ROOT}/include \
	-I${ROOT_SRC}/chrono/include -I${ROOT_SRC}/peripherals/include -I${ROOT_SRC}/font/include -I${ROOT_SRC}/zlib/include \
	-I${ROOT_SRC}/json/include -I${ROOT_SRC}/io/include \
	-Iinclude 

LDFLAGS = -static -L${LIB} -L${SDL}/lib -L${SDL_IMG}/lib -L/msys64/mingw64/lib -L/msys64/mingw64/x86_64-w64-mingw32/lib
LDLIBS = -lchrono -lgraphics -lfilesystem -lfont -ljson -lio -lcore -lzlib -lmingw32 -lSDL2_image -lSDL2main -lSDL2 -mwindows -mconsole -Wl,--no-undefined -Wl,--dynamicbase -Wl,--nxcompat -Wl,--high-entropy-va -lm -ldinput8 -ldxguid -ldxerr8 -luser32 -lgdi32 -lwinmm -limm32 -lole32 -loleaut32 -lshell32 -lsetupapi -lversion -luuid -static-libgcc
OBJS = main.o \
	${ROOT}/src/PicoClock.o \
	${ROOT}/src/Painter.o \
	src/SdlTimer.o \
	src/FileStream.o \
	src/SdlInput.o \
	src/SdlHelper.o \
	src/SdlScreen.o

MAIN = ${BIN}/main
# MAIN = main

# $(MAIN): $(OBJS)
$(MAIN): $(OBJS) executable

$(OBJS): depend

depend:
	$(MAKE) -C ${ROOT_SRC}/core \
	&& $(MAKE) -C ${ROOT_SRC}/zlib \
	&& $(MAKE) -C ${ROOT_SRC}/filesystem \
	&& $(MAKE) -C ${ROOT_SRC}/io \
	&& $(MAKE) -C ${ROOT_SRC}/graphics \
	&& $(MAKE) -C ${ROOT_SRC}/font \
	&& $(MAKE) -C ${ROOT_SRC}/json \
	&& $(MAKE) -C ${ROOT_SRC}/chrono

executable: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LDLIBS) -o ${BIN}/main.exe

.PHONY: clean clean-all

all:	$(MAIN)

$(MAIN): $(OBJS) 
	$(AR) $(ARFLAGS) $(MAIN) $(OBJS)

# this is a suffix replacement rule for building `.o`s from `.c`s
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)
.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean-all: clean 
	$(MAKE) -C ${ROOT_SRC}/core clean \
	&& $(MAKE) -C ${ROOT_SRC}/zlib clean \
	&& $(MAKE) -C ${ROOT_SRC}/filesystem clean \
	&& $(MAKE) -C ${ROOT_SRC}/font clean \
	&& $(MAKE) -C ${ROOT_SRC}/graphics clean \
	&& $(MAKE) -C ${ROOT_SRC}/io clean \
	&& $(MAKE) -C ${ROOT_SRC}/json clean \
	&& $(MAKE) -C ${ROOT_SRC}/chrono clean

clean:
	$(RM) $(OBJS) $(MAIN)

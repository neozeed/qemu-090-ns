include config.mak

TARGET_BASE_ARCH:=$(TARGET_ARCH)
ifeq ($(TARGET_ARCH), x86_64)
TARGET_BASE_ARCH:=i386
endif
ifeq ($(TARGET_ARCH), ppc64)
TARGET_BASE_ARCH:=ppc
endif
ifeq ($(TARGET_ARCH), sparc64)
TARGET_BASE_ARCH:=sparc
endif
TARGET_PATH=$(SRC_PATH)/target-$(TARGET_BASE_ARCH)
VPATH=$(SRC_PATH):$(TARGET_PATH):$(SRC_PATH)/hw:$(SRC_PATH)/audio
CPPFLAGS=-I. -I.. -I$(TARGET_PATH) -I$(SRC_PATH)
ifdef CONFIG_DARWIN_USER
VPATH+=:$(SRC_PATH)/darwin-user
CPPFLAGS+=-I$(SRC_PATH)/darwin-user -I$(SRC_PATH)/darwin-user/$(TARGET_ARCH)
endif
ifdef CONFIG_LINUX_USER
VPATH+=:$(SRC_PATH)/linux-user
CPPFLAGS+=-I$(SRC_PATH)/linux-user -I$(SRC_PATH)/linux-user/$(TARGET_ARCH)
endif
BASE_CFLAGS=
BASE_LDFLAGS=
#CFLAGS+=-Werror
LIBS=
HELPER_CFLAGS=$(CFLAGS)
DYNGEN=../dyngen$(EXESUF)
# user emulator name
TARGET_ARCH2=$(TARGET_ARCH)
ifeq ($(TARGET_ARCH),arm)
  ifeq ($(TARGET_WORDS_BIGENDIAN),yes)
    TARGET_ARCH2=armeb
  endif
endif
ifeq ($(TARGET_ARCH),sh4)
  ifeq ($(TARGET_WORDS_BIGENDIAN),yes)
    TARGET_ARCH2=sh4eb
  endif
endif
ifeq ($(TARGET_ARCH),mips)
  ifneq ($(TARGET_WORDS_BIGENDIAN),yes)
    TARGET_ARCH2=mipsel
  endif
endif
QEMU_USER=qemu-$(TARGET_ARCH2)
# system emulator name
ifdef CONFIG_SOFTMMU
ifeq ($(TARGET_ARCH), i386)
QEMU_SYSTEM=qemu$(EXESUF)
else
QEMU_SYSTEM=qemu-system-$(TARGET_ARCH2)$(EXESUF)
endif
else
QEMU_SYSTEM=qemu-fast
endif

ifdef CONFIG_USER_ONLY
PROGS=$(QEMU_USER)
else
PROGS+=$(QEMU_SYSTEM)
ifndef CONFIG_SOFTMMU
CONFIG_STATIC=y
endif
endif # !CONFIG_USER_ONLY

ifdef CONFIG_STATIC
BASE_LDFLAGS+=-static
endif

# We require -O2 to avoid the stack setup prologue in EXIT_TB
OP_CFLAGS = -Wall -O2 -g -fno-strict-aliasing

ifeq ($(ARCH),i386)
HELPER_CFLAGS+=-fomit-frame-pointer
OP_CFLAGS+=-mpreferred-stack-boundary=2 -fomit-frame-pointer
ifeq ($(HAVE_GCC3_OPTIONS),yes)
OP_CFLAGS+= -falign-functions=0 -fno-gcse
else
OP_CFLAGS+= -malign-functions=0
endif
ifdef TARGET_GPROF
USE_I386_LD=y
endif
ifdef CONFIG_STATIC
USE_I386_LD=y
endif
ifdef USE_I386_LD
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
else
ifdef CONFIG_LINUX_USER
# WARNING: this LDFLAGS is _very_ tricky : qemu is an ELF shared object
# that the kernel ELF loader considers as an executable. I think this
# is the simplest way to make it self virtualizable!
BASE_LDFLAGS+=-Wl,-shared
endif
endif
endif

ifeq ($(ARCH),x86_64)
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif

ifeq ($(ARCH),ppc)
CPPFLAGS+= -D__powerpc__
ifdef CONFIG_LINUX_USER
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif
endif

ifeq ($(ARCH),s390)
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif

ifeq ($(ARCH),sparc)
ifeq ($(CONFIG_SOLARIS),yes)
BASE_CFLAGS+=-mcpu=ultrasparc -m32 -ffixed-g2 -ffixed-g3
BASE_LDFLAGS+=-m32
OP_CFLAGS+=-fno-delayed-branch -fno-omit-frame-pointer -ffixed-i0
else
BASE_CFLAGS+=-mcpu=ultrasparc -m32 -ffixed-g1 -ffixed-g2 -ffixed-g3 -ffixed-g6
BASE_LDFLAGS+=-m32
OP_CFLAGS+=-fno-delayed-branch -ffixed-i0
HELPER_CFLAGS=$(CFLAGS) -ffixed-i0 -mflat
# -static is used to avoid g1/g3 usage by the dynamic linker
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld -static
endif
endif

ifeq ($(ARCH),sparc64)
BASE_CFLAGS+=-mcpu=ultrasparc -m64 -ffixed-g1 -ffixed-g4 -ffixed-g5 -ffixed-g7
BASE_LDFLAGS+=-m64
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
OP_CFLAGS+=-mcpu=ultrasparc -m64 -ffixed-g1 -ffixed-g4 -ffixed-g5 -ffixed-g7 -fno-delayed-branch -ffixed-i0
endif

ifeq ($(ARCH),alpha)
# -msmall-data is not used for OP_CFLAGS because we want two-instruction
# relocations for the constant constructions
# Ensure there's only a single GP
BASE_CFLAGS+=-msmall-data
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif

ifeq ($(ARCH),ia64)
BASE_CFLAGS+=-mno-sdata
OP_CFLAGS+=-mno-sdata
BASE_LDFLAGS+=-Wl,-G0 -Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif

ifeq ($(ARCH),arm)
OP_CFLAGS+=-mno-sched-prolog -fno-omit-frame-pointer
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif

ifeq ($(ARCH),m68k)
OP_CFLAGS+=-fomit-frame-pointer
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif

ifeq ($(ARCH),mips)
BASE_LDFLAGS+=-Wl,-T,$(SRC_PATH)/$(ARCH).ld
endif

ifeq ($(HAVE_GCC3_OPTIONS),yes)
# very important to generate a return at the end of every operation
OP_CFLAGS+=-fno-reorder-blocks -fno-optimize-sibling-calls
endif

ifeq ($(CONFIG_DARWIN),yes)
LIBS+=-lmx
endif

ifdef CONFIG_DARWIN_USER
# Leave some space for the regular program loading zone
BASE_LDFLAGS+=-Wl,-segaddr,__STD_PROG_ZONE,0x1000 -image_base 0x0e000000
endif

OP_CFLAGS+=$(OS_CFLAGS)

#########################################################

CPPFLAGS+=-D_GNU_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE
LIBS+=-lm
ifndef CONFIG_USER_ONLY
LIBS+=-lz
endif
ifdef CONFIG_WIN32
LIBS+=-lwinmm -lws2_32 -liphlpapi
endif
ifdef CONFIG_SOLARIS
LIBS+=-lsocket -lnsl -lresolv
endif

# profiling code
ifdef TARGET_GPROF
BASE_LDFLAGS+=-p
main.o: BASE_CFLAGS+=-p
endif

ifdef CONFIG_LINUX_USER
OBJS= main.o syscall.o mmap.o signal.o path.o osdep.o thunk.o \
      elfload.o linuxload.o
ifdef TARGET_HAS_BFLT
OBJS+= flatload.o
endif

ifeq ($(TARGET_ARCH), i386)
OBJS+= vm86.o
endif
ifeq ($(TARGET_ARCH), arm)
OBJS+=nwfpe/fpa11.o nwfpe/fpa11_cpdo.o \
nwfpe/fpa11_cpdt.o nwfpe/fpa11_cprt.o nwfpe/fpopcode.o nwfpe/single_cpdo.o \
 nwfpe/double_cpdo.o nwfpe/extended_cpdo.o arm-semi.o
endif
ifeq ($(TARGET_ARCH), m68k)
OBJS+= m68k-sim.o m68k-semi.o
endif
endif #CONFIG_LINUX_USER

ifdef CONFIG_DARWIN_USER
OBJS= main.o commpage.o machload.o mmap.o osdep.o signal.o syscall.o thunk.o
endif

SRCS:= $(OBJS:.o=.c)
OBJS+= libqemu.a

# cpu emulator library
LIBOBJS=exec.o kqemu.o translate-op.o translate-all.o cpu-exec.o\
        translate.o op.o 
ifdef CONFIG_SOFTFLOAT
LIBOBJS+=fpu/softfloat.o
else
LIBOBJS+=fpu/softfloat-native.o
endif
CPPFLAGS+=-I$(SRC_PATH)/fpu

ifeq ($(TARGET_ARCH), i386)
LIBOBJS+=helper.o helper2.o
ifeq ($(ARCH), i386)
LIBOBJS+=translate-copy.o
endif
endif

ifeq ($(TARGET_ARCH), x86_64)
LIBOBJS+=helper.o helper2.o
endif

ifeq ($(TARGET_BASE_ARCH), ppc)
LIBOBJS+= op_helper.o helper.o
endif

ifeq ($(TARGET_ARCH), mips)
LIBOBJS+= op_helper.o helper.o
endif

ifeq ($(TARGET_BASE_ARCH), sparc)
LIBOBJS+= op_helper.o helper.o
endif

ifeq ($(TARGET_BASE_ARCH), arm)
LIBOBJS+= op_helper.o helper.o
endif

ifeq ($(TARGET_BASE_ARCH), sh4)
LIBOBJS+= op_helper.o helper.o
endif

ifeq ($(TARGET_BASE_ARCH), m68k)
LIBOBJS+= helper.o
endif

# NOTE: the disassembler code is only needed for debugging
LIBOBJS+=disas.o 
ifeq ($(findstring i386, $(TARGET_ARCH) $(ARCH)),i386)
USE_I386_DIS=y
endif
ifeq ($(findstring x86_64, $(TARGET_ARCH) $(ARCH)),x86_64)
USE_I386_DIS=y
endif
ifdef USE_I386_DIS
LIBOBJS+=i386-dis.o
endif
ifeq ($(findstring alpha, $(TARGET_ARCH) $(ARCH)),alpha)
LIBOBJS+=alpha-dis.o
endif
ifeq ($(findstring ppc, $(TARGET_BASE_ARCH) $(ARCH)),ppc)
LIBOBJS+=ppc-dis.o
endif
ifeq ($(findstring mips, $(TARGET_ARCH) $(ARCH)),mips)
LIBOBJS+=mips-dis.o
endif
ifeq ($(findstring sparc, $(TARGET_BASE_ARCH) $(ARCH)),sparc)
LIBOBJS+=sparc-dis.o
endif
ifeq ($(findstring arm, $(TARGET_ARCH) $(ARCH)),arm)
LIBOBJS+=arm-dis.o
endif
ifeq ($(findstring m68k, $(TARGET_ARCH) $(ARCH)),m68k)
LIBOBJS+=m68k-dis.o
endif
ifeq ($(findstring sh4, $(TARGET_ARCH) $(ARCH)),sh4)
LIBOBJS+=sh4-dis.o
endif

ifdef CONFIG_GDBSTUB
OBJS+=gdbstub.o
endif

all: $(PROGS)

$(QEMU_USER): $(OBJS)
	$(CC) $(LDFLAGS) $(BASE_LDFLAGS) -o $@ $^  $(LIBS)
ifeq ($(ARCH),alpha)
# Mark as 32 bit binary, i. e. it will be mapped into the low 31 bit of
# the address space (31 bit so sign extending doesn't matter)
	echo -ne '\001\000\000\000' | dd of=qemu bs=1 seek=48 count=4 conv=notrunc
endif

# must use static linking to avoid leaving stuff in virtual address space
VL_OBJS=vl.o osdep.o readline.o monitor.o pci.o console.o loader.o isa_mmio.o
VL_OBJS+=cutils.o
VL_OBJS+=block.o block-raw.o
VL_OBJS+=block-cow.o block-qcow.o aes.o block-vmdk.o block-cloop.o block-dmg.o block-bochs.o block-vpc.o block-vvfat.o block-qcow2.o
ifdef CONFIG_WIN32
VL_OBJS+=tap-win32.o
endif

SOUND_HW = sb16.o es1370.o
AUDIODRV = audio.o noaudio.o wavaudio.o
ifdef CONFIG_SDL
AUDIODRV += sdlaudio.o
endif
ifdef CONFIG_OSS
AUDIODRV += ossaudio.o
endif
ifdef CONFIG_COREAUDIO
AUDIODRV += coreaudio.o
endif
ifdef CONFIG_ALSA
AUDIODRV += alsaaudio.o
LIBS += -lasound
endif
ifdef CONFIG_DSOUND
AUDIODRV += dsoundaudio.o
LIBS += -lole32 -ldxguid
endif
ifdef CONFIG_FMOD
AUDIODRV += fmodaudio.o
audio.o fmodaudio.o: CPPFLAGS := -I$(CONFIG_FMOD_INC) $(CPPFLAGS)
LIBS += $(CONFIG_FMOD_LIB)
endif
ifdef CONFIG_ADLIB
SOUND_HW += fmopl.o adlib.o
endif
AUDIODRV+= wavcapture.o

# SCSI layer
VL_OBJS+= scsi-disk.o cdrom.o lsi53c895a.o

# USB layer
VL_OBJS+= usb.o usb-hub.o usb-linux.o usb-hid.o usb-ohci.o usb-msd.o

# PCI network cards
VL_OBJS+= ne2000.o rtl8139.o pcnet.o

ifeq ($(TARGET_BASE_ARCH), i386)
# Hardware support
VL_OBJS+= ide.o pckbd.o ps2.o vga.o $(SOUND_HW) dma.o $(AUDIODRV)
VL_OBJS+= fdc.o mc146818rtc.o serial.o i8259.o i8254.o pcspk.o pc.o
VL_OBJS+= cirrus_vga.o mixeng.o apic.o parallel.o acpi.o piix_pci.o
VL_OBJS+= usb-uhci.o smbus_eeprom.o busmouse.o
CPPFLAGS += -DHAS_AUDIO
endif
ifeq ($(TARGET_BASE_ARCH), ppc)
VL_OBJS+= ppc.o ide.o pckbd.o ps2.o vga.o $(SOUND_HW) dma.o $(AUDIODRV)
VL_OBJS+= mc146818rtc.o serial.o i8259.o i8254.o fdc.o m48t59.o
VL_OBJS+= ppc_prep.o ppc_chrp.o cuda.o adb.o openpic.o heathrow_pic.o mixeng.o
VL_OBJS+= grackle_pci.o prep_pci.o unin_pci.o
CPPFLAGS += -DHAS_AUDIO
endif
ifeq ($(TARGET_ARCH), mips)
VL_OBJS+= mips_r4k.o mips_malta.o mips_timer.o mips_int.o dma.o vga.o serial.o i8254.o i8259.o
VL_OBJS+= ide.o gt64xxx.o pckbd.o ps2.o fdc.o mc146818rtc.o usb-uhci.o acpi.o
VL_OBJS+= piix_pci.o parallel.o mixeng.o cirrus_vga.o $(SOUND_HW) $(AUDIODRV)
DEFINES += -DHAS_AUDIO
endif
ifeq ($(TARGET_BASE_ARCH), sparc)
ifeq ($(TARGET_ARCH), sparc64)
VL_OBJS+= sun4u.o ide.o pckbd.o ps2.o vga.o apb_pci.o
VL_OBJS+= fdc.o mc146818rtc.o serial.o m48t59.o
VL_OBJS+= cirrus_vga.o parallel.o
else
VL_OBJS+= sun4m.o tcx.o pcnet.o iommu.o m48t59.o slavio_intctl.o
VL_OBJS+= slavio_timer.o slavio_serial.o slavio_misc.o fdc.o esp.o sparc32_dma.o
VL_OBJS+= cs4231.o
endif
endif
ifeq ($(TARGET_BASE_ARCH), arm)
VL_OBJS+= integratorcp.o versatilepb.o ps2.o smc91c111.o arm_pic.o arm_timer.o
VL_OBJS+= arm_boot.o pl011.o pl050.o pl080.o pl110.o pl190.o
VL_OBJS+= versatile_pci.o
VL_OBJS+= arm_gic.o realview.o arm_sysctl.o
VL_OBJS+= arm-semi.o
endif
ifeq ($(TARGET_BASE_ARCH), sh4)
VL_OBJS+= shix.o sh7750.o sh7750_regnames.o tc58128.o
endif
ifdef CONFIG_GDBSTUB
VL_OBJS+=gdbstub.o 
endif
ifdef CONFIG_SDL
VL_OBJS+=sdl.o x_keymap.o
endif
VL_OBJS+=vnc.o
ifdef CONFIG_COCOA
VL_OBJS+=cocoa.o
COCOA_LIBS=-F/System/Library/Frameworks -framework Cocoa -framework IOKit
ifdef CONFIG_COREAUDIO
COCOA_LIBS+=-framework CoreAudio
endif
endif
ifdef CONFIG_SLIRP
CPPFLAGS+=-I$(SRC_PATH)/slirp
SLIRP_OBJS=cksum.o if.o ip_icmp.o ip_input.o ip_output.o \
slirp.o mbuf.o misc.o sbuf.o socket.o tcp_input.o tcp_output.o \
tcp_subr.o tcp_timer.o udp.o bootp.o debug.o tftp.o
VL_OBJS+=$(addprefix slirp/, $(SLIRP_OBJS))
endif

VL_LDFLAGS=
# specific flags are needed for non soft mmu emulator
ifdef CONFIG_STATIC
VL_LDFLAGS+=-static
endif
ifndef CONFIG_SOFTMMU
VL_LDFLAGS+=-Wl,-T,$(SRC_PATH)/i386-vl.ld 
endif
ifndef CONFIG_DARWIN
ifndef CONFIG_WIN32
ifndef CONFIG_SOLARIS
VL_LIBS=-lutil -lrt
endif
endif
endif
ifdef TARGET_GPROF
vl.o: BASE_CFLAGS+=-p
VL_LDFLAGS+=-p
endif

ifeq ($(ARCH),ia64)
VL_LDFLAGS+=-Wl,-G0 -Wl,-T,$(SRC_PATH)/ia64.ld
endif

ifeq ($(ARCH),sparc64)
VL_LDFLAGS+=-m64
VL_LDFLAGS+=-Wl,-T,$(SRC_PATH)/sparc64.ld
endif

ifdef CONFIG_WIN32
SDL_LIBS := $(filter-out -mwindows, $(SDL_LIBS)) -mconsole
endif

$(QEMU_SYSTEM): $(VL_OBJS) libqemu.a
	$(CC) $(VL_LDFLAGS) -o $@ $^ $(LIBS) $(SDL_LIBS) $(COCOA_LIBS) $(VL_LIBS)

cocoa.o: cocoa.m
	$(CC) $(CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) -c -o $@ $<

sdl.o: sdl.c keymaps.c sdl_keysym.h
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SDL_CFLAGS) $(BASE_CFLAGS) -c -o $@ $<

vnc.o: vnc.c keymaps.c sdl_keysym.h vnchextile.h
	$(CC) $(CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) -c -o $@ $<

sdlaudio.o: sdlaudio.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SDL_CFLAGS) $(BASE_CFLAGS) -c -o $@ $<

depend: $(SRCS)
	$(CC) -MM $(CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) $^ 1>.depend

vldepend: $(VL_OBJS:.o=.c)
	$(CC) -MM $(CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) $^ 1>.depend

# libqemu 

libqemu.a: $(LIBOBJS)
	rm -f $@
	$(AR) rcs $@ $(LIBOBJS)

translate.o: translate.c gen-op.h opc.h cpu.h

translate-all.o: translate-all.c opc.h cpu.h

translate-op.o: translate-all.c op.h opc.h cpu.h

op.h: op.o $(DYNGEN)
	$(DYNGEN) -o $@ $<

opc.h: op.o $(DYNGEN)
	$(DYNGEN) -c -o $@ $<

gen-op.h: op.o $(DYNGEN)
	$(DYNGEN) -g -o $@ $<

op.o: op.c
	$(CC) $(OP_CFLAGS) $(CPPFLAGS) -c -o $@ $<

# HELPER_CFLAGS is used for all the code compiled with static register
# variables
ifeq ($(TARGET_BASE_ARCH), i386)
# XXX: rename helper.c to op_helper.c
helper.o: helper.c
	$(CC) $(HELPER_CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) -c -o $@ $<
else
op_helper.o: op_helper.c
	$(CC) $(HELPER_CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) -c -o $@ $<
endif

cpu-exec.o: cpu-exec.c
	$(CC) $(HELPER_CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) -c -o $@ $<

# Note: this is a workaround. The real fix is to avoid compiling
# cpu_signal_handler() in cpu-exec.c.
signal.o: signal.c
	$(CC) $(HELPER_CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) -c -o $@ $<

ifeq ($(TARGET_BASE_ARCH), i386)
op.o: op.c opreg_template.h ops_template.h ops_template_mem.h ops_mem.h ops_sse.h
endif

ifeq ($(TARGET_ARCH), arm)
op.o: op.c op_template.h
pl110.o: pl110_template.h
endif

ifeq ($(TARGET_BASE_ARCH), sparc)
op.o: op.c op_template.h op_mem.h fop_template.h fbranch_template.h
magic_load.o: elf_op.h
endif

ifeq ($(TARGET_BASE_ARCH), ppc)
op.o: op.c op_template.h op_mem.h
op_helper.o: op_helper_mem.h
translate.o: translate.c translate_init.c
endif

ifeq ($(TARGET_ARCH), mips)
op.o: op.c op_template.c fop_template.c op_mem.c
op_helper.o: op_helper_mem.c
endif

loader.o: loader.c elf_ops.h

ifeq ($(TARGET_ARCH), sh4)
op.o: op.c op_mem.c cpu.h
op_helper.o: op_helper.c exec.h cpu.h
helper.o: helper.c exec.h cpu.h
sh7750.o: sh7750.c sh7750_regs.h sh7750_regnames.h cpu.h
shix.o: shix.c sh7750_regs.h sh7750_regnames.h
sh7750_regnames.o: sh7750_regnames.c sh7750_regnames.h sh7750_regs.h
tc58128.o: tc58128.c
endif

$(OBJS) $(LIBOBJS) $(VL_OBJS): config.h ../config-host.h

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(BASE_CFLAGS) -c -o $@ $<

%.o: %.S
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f *.o  *.a *~ $(PROGS) gen-op.h opc.h op.h nwfpe/*.o slirp/*.o fpu/*.o

install: all 
ifneq ($(PROGS),)
	$(INSTALL) -m 755 -s $(PROGS) "$(DESTDIR)$(bindir)"
endif

ifneq ($(wildcard .depend),)
include .depend
endif

ifeq (1, 0)
audio.o sdlaudio.o dsoundaudio.o ossaudio.o wavaudio.o noaudio.o \
fmodaudio.o alsaaudio.o mixeng.o sb16.o es1370.o gus.o adlib.o: \
CFLAGS := $(CFLAGS) -Wall -Werror -W -Wsign-compare
endif

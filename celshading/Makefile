TARGET = celshading
OBJS = lightmap.o celshading.o callbacks.o

INCDIR =
CFLAGS = -G0 -Wall -O2
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
ASFLAGS = $(CFLAGS)

LIBDIR =
LDFLAGS =
LIBS= -lpspdebug -lpsphprm_driver  -lpsprtc   -lpspvfpu -lpspgum -lm -lpspgu 

EXTRA_TARGETS = EBOOT.PBP
PSP_EBOOT_TITLE = Celshading Sample SIO MPU-6050

PSPSDK=$(shell psp-config --pspsdk-path)
include $(PSPSDK)/lib/build.mak

lightmap.o : lightmap.raw
	bin2o -i lightmap.raw lightmap.o lightmap

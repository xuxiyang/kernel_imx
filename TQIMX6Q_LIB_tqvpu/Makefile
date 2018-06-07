#arm-linux-gnueabihf-gcc -std=99
CC=arm-linux-gnueabihf-gcc
LINK=arm-linux-gnueabihf-gcc

 

#ifneq ($(wildcard lib/libtqvpu.so),)

#else
OBJ-vpu = vpu/dec.c \
	  vpu/enc.c \
      vpu/capture.c \
      vpu/display.c \
      vpu/fb.c \
      vpu/utils.c	  

#endif

OBJ-dec = decode_test.o \
	  ipu/lcd.o \
	  ipu/ipu_test.o
	  
OBJ-enc = encode_test.o \
	  ipu/lcd.o \
	  ipu/ipu_test.o

OBJ-decthread = decode_thread.o \
	  ipu/lcd.o \
	  ipu/ipu_test.o
	  
TARGET-dec = decode_test.out
TARGET-decthread = decode_thread.out
TARGET-enc = encode_test.out
TARGET-vpu = lib/libtqvpu.so

all: $(TARGET-vpu) \
	$(TARGET-dec) \
	$(TARGET-decthread) \
	$(TARGET-enc)


$(TARGET-vpu): $(OBJ-vpu)
ifneq ($(wildcard lib/libtqipu.so),)

else
	$(CC) -fPIC -shared -o lib/libtqipu.so ipu/tq_ipu.c 
endif

ifneq ($(wildcard lib/libtqvpu.so),)

else
	$(CC) $(OBJ-vpu) -fPIC -shared -o $(TARGET-vpu) 	
endif	

$(TARGET-dec): $(OBJ-dec)
	$(CC) -o $(TARGET-dec) $(OBJ-dec) ${LIBS} -lrt -lpthread -L ./lib -lvpu -lipu -ltqipu -ltqvpu	

$(TARGET-decthread): $(OBJ-decthread)
	$(CC) -o $(TARGET-decthread) $(OBJ-decthread) ${LIBS} -lrt -lpthread -L ./lib -lvpu -lipu -ltqipu -ltqvpu	

	
$(TARGET-enc): $(OBJ-enc)
	$(CC) -o $(TARGET-enc) $(OBJ-enc) ${LIBS} -lrt -lpthread -L ./lib -lvpu -lipu -ltqipu -ltqvpu	


%.o: %.c
	$(CC) $(INC) -Wall -O2 -c $^ -o $@

.PHONY: clean
clean:
	rm -f $(OBJDIR)/$(TARGET-dec) $(OBJ-dec) \
	$(TARGET-decthread) $(OBJ-decthread)\
	$(TARGET-enc) $(OBJ-enc)\
	$(TARGET-vpu) $(OBJ-vpu)\

	
	



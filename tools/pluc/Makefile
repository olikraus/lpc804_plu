# required subdirs:
# app
# cube
# util 
# cfg: contains only config.h
# gnet: only dgd_opt.h
# gnet & syl only required for dgc

CFLAGS = -g -Wall -ffunction-sections -fdata-sections -I. 
LDFLAGS = -Wl,--gc-sections


SRC = $(shell ls *.c)  

OBJ = $(SRC:.c=.o)

all: pluc
	
pluc: $(OBJ) 
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -o pluc -lm
	
clean:
	-rm $(OBJ) pluc
	
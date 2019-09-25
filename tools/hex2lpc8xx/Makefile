# 
# hex2lpc8xx: Linux flash utility for LPC8xx devices
# 

#CFLAGS = -g  -pg -Wall 
CFLAGS = -O4 -Wall

SRC = hex2lpc8xx.c

OBJ = $(SRC:.c=.o)

hex2lpc: $(SRC)
	$(CC) -Wall $(LDFLAGS) $(SRC) -o hex2lpc8xx

clean:	
	-rm ./hex2lpc8xx


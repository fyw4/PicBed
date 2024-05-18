#
# Makefile
#

INC_PATH = -I ./include
LDFLAGS  = -L ./lib

all:cantest 

cantest:cantest.c
	$(CC) $(INC_PATH) $(LDFLAGS) -Wall $< -o $@ -lsocketcan
	
clean:
	rm -f cantest 

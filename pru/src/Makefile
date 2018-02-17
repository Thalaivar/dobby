ODIR= ../obj
LDIR= ../lib
BDIR= ../bin

CC= gcc
CFLAGS= -I$(LDIR)
CSRC= $(wildcard *.c)
OBJ= $(CSRC:.c=.o)

LDFLAGS= -lpthread -lprussdrv
DEPS= $(wildcard $(LDIR)/ *.h)

$(ODIR)/%.o: %.c $(DEPS)
		$(CC) -c -o $@ $< $(CFLAGS) $(LDFLAGS)

test_dobby_pwm: $(OBJ)
		$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)
		rm *.o

.PHONY: clean

clean:
		rm -f *.o test_dobby_pwm
		rm -f $(ODIR)/*.o

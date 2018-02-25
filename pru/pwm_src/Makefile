CC = g++
LIBS = -lpthread -lprussdrv
CFLAGS = -o

CSRC = pwm.cpp
CSRC_ESC = calibrate_esc.cpp

TARGETS = calibrate_esc

$(TARGETS): $(CSRC) $(CSRC_ESC)
	$(CC) $^ $(CFLAGS) $@ $(LIBS)

.PHONY: clean

clean:
	rm $(TARGETS)

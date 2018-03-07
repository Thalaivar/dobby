CC = g++
LIBS = -lpthread -lprussdrv
CFLAGS = -o

CSRC = ppm.cpp
CSRC_ESC = test_decoder_pru.cpp

TARGETS = ppm_decode

$(TARGETS): $(CSRC) $(CSRC_ESC)
	$(CC) $^ $(CFLAGS) $@ $(LIBS)

.PHONY: clean

clean:
	rm $(TARGETS)

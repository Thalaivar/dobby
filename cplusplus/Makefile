CC = g++-6
LIB = lib
SRC = src
LFLAGS = -lroboticscape -lprussdrv -lrt -Wall
CFLAGS = -I$(LIB)/

DEPS = $(wildcard $(LIB)/*.h)
CSRC = $(wildcard $(SRC)/*.cpp)
OBJ = $(CSRC:.cpp=.o)

TARGET = main

$(TARGET): $(OBJ) $(TARGET).o
	@$(CC)  $^ -o $@ $(LFLAGS) $(CFLAGS)
	@rm *.o
	@echo "****************************"
	@echo "$(TARGET).cpp compiled!"
	@echo "****************************"

%.o: %.cpp $(DEPS)
	@$(CC) -c $< -o $@ $(LFLAGS) $(CFLAGS)
	@echo "Compiled: "$<

.PHONY: clean

clean:
	@rm -f main
	@rm $(OBJ)
	@echo "$(TARGET) clean completed!"

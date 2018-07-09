CC = g++-6
LIB = lib
SRC = src
LFLAGS = -lprussdrv -lrt -Wall -l:librobotcontrol.so.1 -pthread -lm
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

imu_test: $(OBJ) imu_test.o
	@$(CC) $^ -o $@ $(LFLAGS) $(CFLAGS)
	@rm imu_test.o
	@echo "****************************"
	@echo "imu_test.cpp compiled!"
	@echo "****************************"



%.o: %.cpp $(DEPS)
	@$(CC) -c $< -o $@ $(LFLAGS) $(CFLAGS)
	@echo "Compiled: "$<

.PHONY: clean

clean:
	@rm -f main
	@rm $(OBJ)
	@echo "$(TARGET) clean completed!"

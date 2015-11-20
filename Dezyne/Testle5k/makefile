TARGET = main
MAINPATH = .
CC = g++

CPP_FILES := $(wildcard $(MAINPATH)/*.cc)
CPP_FILES += $(wildcard $(MAINPATH)/*.cpp)

#SRC += $(TARGET).cc
SRC += $(CPP_FILES)

OBJ = $(TARGET).o

CFLAGS += -std=c++11
CFLAGS += -I $(MAINPATH)

%.o: %.cpp
	@echo "Compiling .cpp -> .o"
	$(CC) $(SRC) -o $@ $(CFLAGS)

main:
	@echo "Compiling main" 
	$(CC) $(SRC) -o $(OBJ) $(CFLAGS)

clean:
	@echo "Removing all .o files" 
	rm -rf *.o

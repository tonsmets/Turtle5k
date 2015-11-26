TARGET = main_prog
MAINPATH = .
INTERFACE_IMPL_PATH = $(MAINPATH)/Interface_impls
DEZYNE_FRAMEWORK_PATH = $(MAINPATH)/Dezyne_framework
GENERATED_CODE_PATH = $(MAINPATH)/Generated_code

CC = g++

CPP_FILES := $(wildcard $(MAINPATH)/*.c*)
CPP_FILES += $(wildcard $(INTERFACE_IMPL_PATH)/*.c*)
CPP_FILES += $(wildcard $(DEZYNE_FRAMEWORK_PATH)/*.c*)
CPP_FILES += $(wildcard $(GENERATED_CODE_PATH)/*.c*)

#SRC += $(TARGET).cc
SRC += $(CPP_FILES)

OBJ = $(TARGET).o

CFLAGS += -std=c++11
CFLAGS += -I $(MAINPATH)
CFLAGS += -I $(INTERFACE_IMPL_PATH)
CFLAGS += -I $(DEZYNE_FRAMEWORK_PATH)
CFLAGS += -I $(GENERATED_CODE_PATH)

%.o: %.cpp
	@echo "Compiling .cpp -> .o"
	$(CC) $(SRC) -o $@ $(CFLAGS)

main:
	@echo "Compiling main" 
	$(CC) $(SRC) -o $(OBJ) $(CFLAGS)

clean:
	@echo "Removing all .o files" 
	rm -rf *.o

################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../ADC.c \
../Compass.c \
../Digital_Input.c \
../Digital_Output.c \
../UART.c \
../Shoot_Level.c \
../Shoot_Puls.c \
../main.c \
../Debug_UART.c \
../usart_driver.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
ADC.o \
Compass.o \
Digital_Input.o \
Digital_Output.o \
UART.o \
Shoot_Level.o \
Shoot_Puls.o \
main.o \
Debug_UART.o \
usart_driver.o

OBJS_AS_ARGS +=  \
ADC.o \
Compass.o \
Digital_Input.o \
Digital_Output.o \
UART.o \
Shoot_Level.o \
Shoot_Puls.o \
main.o \
Debug_UART.o \
usart_driver.o

C_DEPS +=  \
ADC.d \
Compass.d \
Digital_Input.d \
Digital_Output.d \
UART.d \
Shoot_Level.d \
Shoot_Puls.d \
main.d \
Debug_UART.d \
usart_driver.d

C_DEPS_AS_ARGS +=  \
ADC.d \
Compass.d \
Digital_Input.d \
Digital_Output.d \
UART.d \
Shoot_Level.d \
Shoot_Puls.d \
main.d \
Debug_UART.d \
usart_driver.d

OUTPUT_FILE_PATH +=Turtle_5K_IO_Board.elf

OUTPUT_FILE_PATH_AS_ARGS +=Turtle_5K_IO_Board.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler





















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG -DF_CPU=16000000 -DCONFIG_NVM_IGNORE_XMEGA_A3_D3_REVB_ERRATA -DBOARD=USER_BOARD  -I".." -I"../asf" -I"../asf/xmega/utils" -I"../asf/xmega/utils/preprocessor" -I"../asf/xmega/drivers/cpu" -I"../asf/xmega/services/basic/sleepmgr" -I"../asf/xmega/drivers/sleep" -I"../asf/xmega/boards" -I"../asf/xmega/services/basic/clock"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atxmega64a3 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="Turtle_5K_IO_Board.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atxmega64a3  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Turtle_5K_IO_Board.elf" "Turtle_5K_IO_Board.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "Turtle_5K_IO_Board.elf" "Turtle_5K_IO_Board.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "Turtle_5K_IO_Board.elf" > "Turtle_5K_IO_Board.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "Turtle_5K_IO_Board.elf" "Turtle_5K_IO_Board.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-size.exe" "Turtle_5K_IO_Board.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Turtle_5K_IO_Board.elf" "Turtle_5K_IO_Board.a" "Turtle_5K_IO_Board.hex" "Turtle_5K_IO_Board.lss" "Turtle_5K_IO_Board.eep" "Turtle_5K_IO_Board.map" "Turtle_5K_IO_Board.srec" "Turtle_5K_IO_Board.usersignatures"
	
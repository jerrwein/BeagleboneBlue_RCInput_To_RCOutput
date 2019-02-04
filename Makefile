TARGET = RCInput_To_RCOutput

all: $(TARGET)

$(TARGET): $(TARGET).o AM335X_GPIO.o
	@echo 'Linking target: $@'
	@echo 'Invoking: GCC Linker'
	gcc -Wall -O2 $(TARGET).o AM335X_GPIO.o -o $(TARGET)
	@echo 'Finished building target: $@'
	@echo ' '

$(TARGET).o: $(TARGET).c
	@echo 'Compiling target: $@'
	@echo 'Invoking: GCC  compiler'
	gcc -Wall -O2 -c -o "$@" "$<"
	@echo 'Finished building target: $@'
	@echo ' '


AM335X_GPIO.o: AM335X_GPIO.c
	@echo 'Compiling target: $@'
	@echo 'Invoking: GCC  compiler'
	gcc -Wall -O2 -c -o "$@" "$<"
	@echo 'Finished building target: $@'
	@echo ' '

clean:
	$(RM) $(TARGET) $(TARGET).o AM335X_GPIO.o

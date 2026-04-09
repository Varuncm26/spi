# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -O2 -I.

# Target executable name
TARGET = mpu6500_app

# Source files
SRCS = main.c spi.c mpu6500.c

# Object files (automatically generated from SRCS)
OBJS = $(SRCS:.c=.o)

# Header files (for dependency tracking)
HEADERS = spi.h mpu6500.h mpuRegister.h

# Default rule
all: $(TARGET)

# Link object files to create the executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS)

# Compile C source files into object files
# This rule also ensures that if a header changes, the files are recompiled
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

# Mark rules that don't represent physical files
.PHONY: all clean

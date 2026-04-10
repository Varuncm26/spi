# Makefile — MPU6500 3D OpenCV Visualizer (Raspberry Pi 5)
#
# Install dependency first:
#   sudo apt update && sudo apt install libopencv-dev
#
# Build:
#   make
#
# Run:
#   sudo ./imu_viz          (sudo needed for /dev/spidev access)
#   OR add your user to the spi group and run without sudo:
#   sudo usermod -aG spi $USER  (then log out and back in)

CXX     := g++
CC      := gcc
TARGET  := imu_viz

# All source files
SRCS_CPP := viz_main.cpp
SRCS_C   := mpu6500.c spi.c

OBJS_CPP := $(SRCS_CPP:.cpp=.o)
OBJS_C   := $(SRCS_C:.c=.o)
OBJS     := $(OBJS_CPP) $(OBJS_C)

# Flags
CFLAGS   := -O2 -Wall -Wextra
CXXFLAGS := -O2 -Wall -Wextra -std=c++17
OPENCV   := $(shell pkg-config --cflags --libs opencv4 2>/dev/null || \
                     pkg-config --cflags --libs opencv  2>/dev/null)

ifeq ($(OPENCV),)
  $(error "OpenCV not found. Run: sudo apt install libopencv-dev")
endif

LDFLAGS  := $(OPENCV) -lm

# Build rules
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)
	@echo "Build OK → ./$(TARGET)"

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(OPENCV) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: clean

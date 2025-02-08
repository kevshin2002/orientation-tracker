CXX := g++
CXXFLAGS := -std=c++17 -g -Wall

BUILD_DIR := build
SRC := main.cpp 
EXEC := IMU

# Default target (Release build)
all: release

# Build in Release mode
release: clean
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Release
	make -C $(BUILD_DIR) -j$(nproc)

# Build in Debug mode
debug: clean
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Debug
	make -C $(BUILD_DIR) -j$(nproc)

# Clean build files
clean:
	rm -rf $(BUILD_DIR)


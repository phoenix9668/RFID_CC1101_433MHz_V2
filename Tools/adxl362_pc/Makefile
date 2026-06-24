CC ?= cc
ROOT := ../..
BUILD_DIR ?= build
TARGET := $(BUILD_DIR)/adxl362_pc
PC_OBJECTS := $(BUILD_DIR)/main.o $(BUILD_DIR)/pc_platform.o
FIRMWARE_OBJECT := $(BUILD_DIR)/adxl362.o
OBJECTS := $(PC_OBJECTS) $(FIRMWARE_OBJECT)
CPPFLAGS := -Iinclude -I$(ROOT)/Core/Inc -D_Original_Data_Algorithm=1 -DADXL362_PC_RUNNER=1
CFLAGS := -std=c11 -O2
PC_WARNINGS := -Wall -Wextra -Wpedantic -Wconversion -Wshadow
FIRMWARE_WARNINGS := -Wall -Wextra
LDLIBS := -lm
SANITIZE_CFLAGS := -std=c11 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer
SANITIZE_LDFLAGS := -fsanitize=address,undefined

.PHONY: all clean sanitize

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@

$(BUILD_DIR)/main.o: src/main.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(PC_WARNINGS) -c $< -o $@

$(BUILD_DIR)/pc_platform.o: src/pc_platform.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(PC_WARNINGS) -c $< -o $@

$(FIRMWARE_OBJECT): $(ROOT)/Core/Src/adxl362_behavior.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(FIRMWARE_WARNINGS) -c $< -o $@

sanitize:
	$(MAKE) BUILD_DIR=build-sanitize CFLAGS="$(SANITIZE_CFLAGS)" \
		LDFLAGS="$(SANITIZE_LDFLAGS)" all

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf build build-sanitize

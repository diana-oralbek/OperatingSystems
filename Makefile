CC      = gcc
TARGET  = mars_rover

FREERTOS_DIR = FreeRTOS-Kernel
POSIX_PORT   = $(FREERTOS_DIR)/portable/ThirdParty/GCC/Posix

SRCS = \
	src/main.c \
	src/comms.c \
	src/motor.c \
	src/navigation.c \
	src/power.c \
	src/self_monitor.c \
	src/sensor.c \
	src/system.c \
	src/main_computer.c \
	$(FREERTOS_DIR)/tasks.c \
	$(FREERTOS_DIR)/queue.c \
	$(FREERTOS_DIR)/list.c \
	$(FREERTOS_DIR)/timers.c \
	$(FREERTOS_DIR)/event_groups.c \
	$(FREERTOS_DIR)/stream_buffer.c \
	$(FREERTOS_DIR)/portable/MemMang/heap_4.c \
	$(POSIX_PORT)/port.c \
	$(POSIX_PORT)/utils/wait_for_event.c

CFLAGS = \
	-Wall \
	-I$(FREERTOS_DIR)/include \
	-I$(POSIX_PORT) \
	-Isrc

LDFLAGS = -lpthread

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)

CC = gcc
OBJS = MtkFlash.o
TARGET = MtkFlash


all: $(TARGET)

$(TARGET): $(OBJS)

clean:
	rm $(OBJS) $(TARGET)

.PHONY: all clean

CC=g++
LDFLAGS=-std=c++11 -O3 -lm -Wall
SOURCES=src/manager.cpp src/main.cpp
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=interchangeManager
INCLUDES=src/manager.h src/item.h

all: $(SOURCES) bin/$(EXECUTABLE)

bin/$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o:  %.c  ${INCLUDES}
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o bin/$(EXECUTABLE)

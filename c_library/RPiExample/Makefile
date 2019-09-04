# General
CC = g++

# Names and flags
INC_DIR = ..
CPPSOURCES += $(wildcard *.cpp $(INC_DIR)/*.cpp)
OBJECTS = $(CPPSOURCES:.cpp=.o)
EXECUTABLE = mpuXX50
CFLAGS = -c -g -Wall -I $(INC_DIR)
LIBS =

# Compile and clean
all: $(CPPSOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIBS)

.cpp.o:
	$(CC) $(CFLAGS) -std=c++14 $(INCLUDES) $< -o $@

clean:
	@echo $OBJECTS = $(OBJECTS)
	rm -rf $(OBJECTS) $(EXECUTABLE)

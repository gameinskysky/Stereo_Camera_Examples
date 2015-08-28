CC=g++
INCLUDE_DIRS=-Iinclude
SRC_DIR=src
BIN_DIR=bin
CFLAGS=-c -Wall -std=c++11 -Wno-unused-function
DFLAGS=-g
INCLUDE_DIRS+=`pkg-config --cflags opencv`
LDFLAGS=`pkg-config --libs opencv`
TARGET=libuvc_cpp_opencv_example


SOURCES=$(wildcard **/*.cpp)



OBJECTS=$(SOURCES:%.cpp=%.o)
OBJECTS=$(patsubst $(SRC_DIR)/%.cpp,$(BIN_DIR)/%.o,$(SOURCES))
DEPS=$(OBJECTS:.o=.d)
all: $(TARGET)

-include $(DEPS)

$(TARGET): $(OBJECTS)
	$(CC) $(INCLUDE_DIRS) $(OBJECTS) -o $(BIN_DIR)/$@ $(LDFLAGS) -L/usr/local/lib -luvc 
$(BIN_DIR)/%.o: $(SRC_DIR)/%.cpp 
	$(CC) $(DFLAGS) $(CFLAGS) -MM -MF $(patsubst %.o,%.d,$@) $(INCLUDE_DIRS) $(INCLUDES) $<
	$(CC) $(DFLAGS) $(CFLAGS) $(INCLUDE_DIRS) $< -o $@
clean:
	rm -f $(OBJECTS) 
	rm -f $(BIN_DIR)/$(TARGET) 
	rm -f $(DEPS)

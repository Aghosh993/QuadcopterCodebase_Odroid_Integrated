CFLAGS=-I./include/ -Wall

CC=g++
RM=rm -f

TARGET=telem_proxy

LIB_SRC=$(wildcard source/*.c)
LIB_CPP_SRC=$(wildcard source/*.cpp)

USER_SRC=$(TARGET).cpp

LIB_OBJ=$(patsubst %.c, %.o, $(LIB_SRC))
LIB_CPP_OBJ=$(patsubst %.cpp, %.o, $(LIB_CPP_SRC))

all: target_out

clean:
	$(RM) $(TARGET) $(TARGET).o $(LIB_OBJ) $(LIB_CPP_OBJ)

%.o: %.c
	$(CC) -c $(ARCHFLAGS) $(CFLAGS) -o $@ $<

%.o: %.cpp
	$(CC) -c $(ARCHFLAGS) $(CFLAGS) -o $@ $<

target_out: $(TARGET).o $(LIB_OBJ) $(LIB_CPP_OBJ)
	$(CC) $(CFLAGS) $(TARGET).o $(LIB_OBJ) -o $(TARGET)

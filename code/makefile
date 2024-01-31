.RECIPEPREFIX +=
CC = g++
CFLAGS = -Wall -lm -lrt -lwiringPi -lpthread

PROG = bin/*
OBJS = obj/*

default:
    mkdir -p bin obj
    $(CC) $(CFLAGS) -c src/MiniProjectA.cpp -o obj/MiniProjectA
    $(CC) $(CFLAGS) obj/MiniProjectA -o bin/MiniProjectA

run:
    sudo ./bin/MiniProjectA

clean:
    rm $(PROG) $(OBJS)

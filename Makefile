.PHONY: test clean

all: pid_example

pid.o: pid.cpp
	g++ -Wall -g -c pid.cpp -o pid.o

pid_example: pid_example.cpp pid.o
	g++ -Wall -g pid_example.cpp pid.o -o pid_example

test: pid_example
	./pid_example

clean:
	rm *.o pid_example

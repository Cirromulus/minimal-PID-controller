.PHONY: test clean

all: pid_example

pid.o: pid.cpp
	g++ -Wall -g -DPID_LOGGING -c pid.cpp -o pid.o

pid_example: pid_example.cpp pid.o
	g++ -Wall -g pid_example.cpp pid.o -o pid_example

test: pid_example
	./pid_example 2> log.csv

clean:
	rm *.o pid_example

all: control 

control:
	gcc Control.c -o Control.out -lm

clean:
	rm *.out

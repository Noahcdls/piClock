
spi-test: ili9341.o main.o
	gcc -o spi-test main.o ili9341.o

main.o:
	gcc main.c -c main.o

ili9341.o:
	gcc -Iinclude -c ./src/ili9341.c

blinky:
	gcc basicGpio.c -o blinky

clean:
	rm *.o
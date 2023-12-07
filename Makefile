
spi-test: ili9341.o main.o
	gcc -o spi-test main.o ili9341.o

main.o: include/ili9341.h
	gcc -c main.c

ili9341.o: src/ili9341.c include/ili9341.h
	gcc -Iinclude -c ./src/ili9341.c

blinky:
	gcc basicGpio.c -o blinky

clean:
	rm *.o

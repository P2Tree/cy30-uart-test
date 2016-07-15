CC = arm-fsl-linux-gnueabi-gcc

cy30_main : cy30_main.o cy30_com.o
	$(CC) -o cy30_main cy30_main.o cy30_com.o

cy30_main.o : cy30_main.c cy30_com.c cy30_com.h
	$(CC) -c cy30_main.c

cy30_com.o : cy30_com.c cy30_com.h
	$(CC) -c cy30_com.c

clean : 
	rm -vf cy30_main cy30_main.o cy30_com.o

.PHONY : clean

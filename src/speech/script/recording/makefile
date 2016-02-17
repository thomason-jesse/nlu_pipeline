all: record.o
	gcc record.o -lpthread -lasound -shared -o record.so

record.o: record.c record.h
	gcc -c -fPIC -lasound -lpthread record.c -o record.o

clean:
	rm *.so
	rm *.o

open:
	gcc open.c -lasound -o open

playback:
	gcc playback.c -lasound -o playback



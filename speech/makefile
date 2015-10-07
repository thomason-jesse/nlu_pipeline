SPHINX = -DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" `pkg-config --cflags --libs pocketsphinx sphinxbase`

all: record.o sphinx.o
	gcc sphinx.o record.o -lasound -lpocketsphinx -lsphinxbase -shared -o speechRecognizer.so

sphinx.o: sphinx.c sphinx.h
	gcc -c -fPIC $(SPHINX) sphinx.c -o sphinx.o

record.o: record.c record.h
	gcc -c -fPIC -lasound $(SPHINX) record.c -o record.o

clean:
	rm *.so
	rm *.o

info:
	gcc info.c -lasound -o info

open:
	gcc open.c -lasound -o open

playback:
	gcc playback.c -lasound -o playback



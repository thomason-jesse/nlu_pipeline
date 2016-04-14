all:
	export PKG_CONFIG_PATH=/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/sphinxbase_install/lib/pkgconfig:/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/pocketsphinx_install/lib/pkgconfig

	export LD_LIBRARY_PATH=/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/sphinxbase_install/lib/:/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/pocketsphinx_install/lib/

	gcc -o sphinx sphinx.c -DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" `pkg-config --cflags --libs pocketsphinx sphinxbase` -I/usr/include/

clean:
	rm *.o


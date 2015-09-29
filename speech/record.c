/*

This example reads from the default PCM device
and writes to standard output for 5 seconds of data.

*/

#include "record.h"

//Used to determine when to record. 
int recording = 0;

void startRecord() {
	recording = 1; 
}

void stopRecord() {
	recording = 0; 
}

int record1600Hz(const char *fout) {
	long loops;
  	int rc;
 	int size;
  	snd_pcm_t *handle;
  	snd_pcm_hw_params_t *params;
  	unsigned int val;
  	int dir;
  	snd_pcm_uframes_t frames;
  	char *buffer;

  	/* Open PCM device for recording (capture). */
  	rc = snd_pcm_open(&handle, "default",
					SND_PCM_STREAM_CAPTURE, 0);
	if (rc < 0) {
    	fprintf(stderr,
            "unable to open pcm device: %s\n",
            snd_strerror(rc));
    	return -1; 
	}

  	/* Allocate a hardware parameters object. */
  	snd_pcm_hw_params_alloca(&params);

  	/* Fill it in with default values. */
  	rc = snd_pcm_hw_params_any(handle, params);

  	if (rc < 0) {
  		fprintf(stderr, "Unable to set default params: %s\n", snd_strerror(rc)); 

		return -1; 
  	}

  	/* Set the desired hardware parameters. */

  	/* Interleaved mode */
  	rc = snd_pcm_hw_params_set_access(handle, params,
                      SND_PCM_ACCESS_RW_INTERLEAVED);

	if (rc < 0) {
		fprintf(stderr, "Unable to set access: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* Signed 16-bit little-endian format */
  	rc = snd_pcm_hw_params_set_format(handle, params,
                              SND_PCM_FORMAT_S16_LE);

	if (rc < 0) {
		fprintf(stderr, "Unable to set format: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* One channel (mono) */
  	rc = snd_pcm_hw_params_set_channels(handle, params, 1);

	if (rc < 0) {
		fprintf(stderr, "Unable to set channels: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* 16000 bits/second (i.e. Hz) sampling rate for Sphinx */
  	val = 16000;
  	
	rc = snd_pcm_hw_params_set_rate_near(handle, params,
                                  &val, &dir);

	if (rc < 0) {
		fprintf(stderr, "Unable to set rate: %s\n", snd_strerror(rc)); 

		return -1; 
	}

  	/* Set period size to 32 frames. */
  	frames = 2048;
  	
	rc = snd_pcm_hw_params_set_period_size_near(handle,
                              params, &frames, &dir);

	if (rc < 0) {
		fprintf(stderr, "Unable to set period: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* Write the parameters to the driver */
  	rc = snd_pcm_hw_params(handle, params);
  	
	if (rc < 0) {
    	fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
    	
		return -1; 
  	}

  	/* Use a buffer large enough to hold one period */
  	snd_pcm_hw_params_get_period_size(params,
                                      &frames, &dir);
  	size = frames * 2; /* 2 bytes/sample, 1 channel */
  	buffer = (char *) malloc(size);

	

	FILE *file = fopen(fout, "w"); 

	if (!file) {
		fprintf(stderr, "Could not open file for sound output!\n");

		return -1; 
	}

	//Waits for recording to start. 
	while (!recording);

 	while (recording) {
    	loops--;
    	rc = snd_pcm_readi(handle, buffer, frames);
    
		if (rc == -EPIPE) {
      		/* EPIPE means overrun */
      		fprintf(stderr, "overrun occurred\n");
      		snd_pcm_prepare(handle);
    	} 
		else if (rc < 0) {
      		fprintf(stderr,
              "error from read: %s\n",
              snd_strerror(rc));
    	} 
		else if (rc != (int)frames) {
      		fprintf(stderr, "short read, read %d frames\n", rc);
    	}
    
		rc = write(fileno(file), buffer, size);
    
		if (rc != size)
      		fprintf(stderr, "short write: wrote %d bytes\n", rc);
  	}

	fclose(file);
  	snd_pcm_drain(handle);
  	snd_pcm_close(handle);
  	free(buffer);

  	return 0;
}


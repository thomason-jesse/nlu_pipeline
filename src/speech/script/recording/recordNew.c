/*

This example reads from the default PCM device
and writes to standard output for 5 seconds of data.

*/

#include "record.h" 

//Used to determine when to record. 
static int recording = 0;
static int interrupted = 0; 
static struct micParams mp; 

void startRecord() {
	recording = 1; 
}

void stopRecord() {
	recording = 0; 
}

void interruptRecord() {
	interrupted = 1; 
}

int isInterrupted() {
	return interrupted; 
}

int initMic(){
	int rc = 0; 

  	/* Open PCM device for recording (capture). */
  	rc = snd_pcm_open(&mp.handle, "default",
					SND_PCM_STREAM_CAPTURE, 0);
	if (rc < 0) {
    	fprintf(stderr,
            "unable to open pcm device: %s\n",
            snd_strerror(rc));
    	return -1; 
	}

  	/* Allocate a hardware parameters object. */
  	snd_pcm_hw_params_alloca(&mp.params);

  	/* Fill it in with default values. */
  	rc = snd_pcm_hw_params_any(mp.handle, mp.params);

  	if (rc < 0) {
  		fprintf(stderr, "Unable to set default params: %s\n", snd_strerror(rc)); 

		return -1; 
  	}

  	/* Set the desired hardware parameters. */

  	/* Interleaved mode */
  	rc = snd_pcm_hw_params_set_access(mp.handle, mp.params,
                      SND_PCM_ACCESS_RW_INTERLEAVED);

	if (rc < 0) {
		fprintf(stderr, "Unable to set access: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* Signed 16-bit little-endian format */
  	rc = snd_pcm_hw_params_set_format(mp.handle, mp.params,
                              SND_PCM_FORMAT_S16_LE);

	if (rc < 0) {
		fprintf(stderr, "Unable to set format: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* One channel (mono) */
  	rc = snd_pcm_hw_params_set_channels(mp.handle, mp.params, 1);

	if (rc < 0) {
		fprintf(stderr, "Unable to set channels: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* 16000 bits/second (i.e. Hz) sampling rate for Sphinx */
  	mp.val = 16000;
  	
	rc = snd_pcm_hw_params_set_rate_near(mp.handle, mp.params,
                                  &mp.val, &mp.dir);

	if (rc < 0) {
		fprintf(stderr, "Unable to set rate: %s\n", snd_strerror(rc)); 

		return -1; 
	}

  	/* Set period size to 32 frames. */
  	mp.frames = 2048;
  	
	rc = snd_pcm_hw_params_set_period_size_near(mp.handle,
                              mp.params, &mp.frames, &mp.dir);

	if (rc < 0) {
		fprintf(stderr, "Unable to set period: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* Write the parameters to the driver */
  	rc = snd_pcm_hw_params(mp.handle, mp.params);
  	
	if (rc < 0) {
    	fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
    	
		return -1; 
  	}

  	/* Use a buffer large enough to hold one period */
  	snd_pcm_hw_params_get_period_size(mp.params,
                                      &mp.frames, &mp.dir);

  	/* 2 bytes/sample, 1 channel */
  	mp.buffer = (int16 *) malloc(mp.frames * sizeof(int16));

	return 0; 
}

void closeMic() {
	if (mp.handle) {
  		snd_pcm_drain(mp.handle);
		snd_pcm_close(mp.handle);
	}

	if (mp.buffer)
  		free(mp.buffer);
}

int record1600Hz(char * fileName) {
	int rc = 0; 
	FILE *file = fopen(fileName, "w");

	if (!file) {
		printf("record.c: Error opening voice.raw!");

		return -1; 
	}

	//Clears buffer in case there is any data left over from last recording. 
	memset((void *)mp.buffer, 0, mp.frames * sizeof(int16)); 

	//Waits for recording to start. 
	while (!recording && !interrupted)
		usleep(70000);

	if (interrupted) {
		fclose(file); 

		return 0; 
	}

	//Readys for recording. 
	if (snd_pcm_state(mp.handle) == SND_PCM_STATE_SETUP)
		snd_pcm_prepare(mp.handle);

 	while (recording) {
    	rc = snd_pcm_readi(mp.handle, (char *)mp.buffer, mp.frames);

		if (rc == -EPIPE) {
      		/* EPIPE means overrun */
      		fprintf(stderr, "overrun occurred\n");
      		snd_pcm_prepare(mp.handle);
    	} 
		else if (rc < 0) {
      		fprintf(stderr,
              "error from read: %s\n",
              snd_strerror(rc));
    	} 
		else if (rc != (int)mp.frames) {
      		//fprintf(stderr, "short read, read %d frames\n", rc);
    	}

   		rc = write(fileno(file), (char *)mp.buffer, mp.frames * sizeof(int16));

		if (rc != mp.size);
      		//fprintf(stderr, "short write: wrote %d bytes\n", rc);
  	}

	//Stops mic from recording and gathers remaining data. 
	if ((rc = snd_pcm_drain(mp.handle)) != 0) {
		fprintf(stderr, "Error draining PCM: %d\n", rc);  
		
		return -1; 
	}

	while ((rc = snd_pcm_readi(mp.handle, (char *)mp.buffer, mp.frames)) > 0)
		write(fileno(file), (char *)mp.buffer, mp.frames * sizeof(int16)); 

	fclose(file);

	return 0;
}

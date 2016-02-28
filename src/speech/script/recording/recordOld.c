/*

This example reads from the default PCM device
and writes to standard output for 5 seconds of data.

*/

#include "record.h" 

#define BUFF_SIZE 128

//Used to determine when to record. 
static int recording = 0;
static int interrupted = 0; 
static struct micParams m1;
static struct micParams m2;
static struct micParams m3;  

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

void initMics() {
	initMic(&m1, "bluetooth", 2);
	//initMic(&m2, "hw:1,0", 1);
	//initMic(&m3, "btheadset", 2);  
}

int initMic(struct micParams *mp, char* dev_name, int num_channels){
	int rc = 0; 

  	/* Open PCM device for recording (capture). */
  	rc = snd_pcm_open(&mp->handle, dev_name,
					SND_PCM_STREAM_CAPTURE, 0);
	if (rc < 0) {
    	fprintf(stderr,
            "unable to open pcm device: %s\n",
            snd_strerror(rc));
    	return -1; 
	}

  	/* Allocate a hardware parameters object. */
  	snd_pcm_hw_params_alloca(&mp->params);

  	/* Fill it in with default values. */
  	rc = snd_pcm_hw_params_any(mp->handle, mp->params);

  	if (rc < 0) {
  		fprintf(stderr, "Unable to set default params: %s\n", snd_strerror(rc)); 

		return -1; 
  	}

  	/* Set the desired hardware parameters. */

  	/* Interleaved mode */
  	rc = snd_pcm_hw_params_set_access(mp->handle, mp->params,
                      SND_PCM_ACCESS_RW_INTERLEAVED);

	if (rc < 0) {
		fprintf(stderr, "Unable to set access: %s\n", snd_strerror(rc));

		return -1; 
	}

  	/* Signed 16-bit little-endian format */
  	rc = snd_pcm_hw_params_set_format(mp->handle, mp->params,
                              SND_PCM_FORMAT_S16_LE);

	if (rc < 0) {
		fprintf(stderr, "Unable to set format: %s\n", snd_strerror(rc));

		return -1; 
	}

	rc = snd_pcm_hw_params_set_rate_resample(mp->handle, mp->params, 16000); 


	if (rc < 0) {
		fprintf(stderr, "Unable to set rate: %s\n", snd_strerror(rc)); 
	}

  	/* One channel (mono) */
  	rc = snd_pcm_hw_params_set_channels(mp->handle, mp->params, num_channels);

	if (rc < 0) {
		fprintf(stderr, "Unable to set channels: %s\n", snd_strerror(rc));

		return -1; 
	}


  	/* Write the parameters to the driver */
  	rc = snd_pcm_hw_params(mp->handle, mp->params);
  	
	if (rc < 0) {
    	fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
    	
		return -1; 
  	}

	snd_pcm_prepare(mp->handle); 

  	/* 2 bytes/sample, 1 channel */
  	mp->buffer = (short *)malloc(BUFF_SIZE * sizeof(short));

	return 0; 
}

void closeMics() {
	//closeMic(&m1); 
	//closeMic(&m2);
	closeMic(&m3);  
}

void closeMic(struct micParams *mp) {
	if (mp->handle) {
  		snd_pcm_drain(mp->handle);
		snd_pcm_close(mp->handle);
	}

	if (mp->buffer)
  		free(mp->buffer);
}

int record1600Hz(char * fileName) {
	int rc = 0; 

	//Clears buffer in case there is any data left over from last recording. 
	//memset((void *)m1.buffer, 0, BUFF_SIZE * sizeof(short)); 
	//memset((void *)m2.buffer, 0, m2.frames * sizeof(int16));

	//Waits for recording to start. 
	while (!recording && !interrupted)
		usleep(70000);

	if (interrupted) {
		//fclose(m1.file); 

		return 0; 
	}

	m1.file = fopen("mic1.raw", "w");
	//m2.file = fopen("mic2.raw", "w"); 
	//m3.file = fopen("mic3.raw", "w"); 

	//if (!m1.file) {
	//	printf("record.c: Error opening voice.raw!");

	//	return -1; 
	//}

	//Readys for recording. 
	//if (snd_pcm_state(m1.handle) == SND_PCM_STATE_SETUP)
	//	snd_pcm_prepare(m1.handle);

	//if (snd_pcm_state(m2.handle) == SND_PCM_STATE_SETUP)
	//	snd_pcm_prepare(m2.handle); 

	printf("About to record\n"); 

 	while (recording) {
		recordFromMic(&m1);

		printf("Recorded from microphone\n");  
		//recordFromMic(&m2); 
		//recordFromMic(&m3); 
  	}

	//drainMic(&m1);
	//drainMic(&m2); 

	fclose(m1.file);
	//fclose(m2.file);
	//fclose(m3.file);  

	printf("\nClosed the microphone\n"); 

	return 0;
}

void recordFromMic(struct micParams *mp) {
    	int rc = snd_pcm_readi(mp->handle, (char *)mp->buffer, BUFF_SIZE * sizeof(short));

	printf("READI\n"); 

		if (rc == -EPIPE) {
      		/* EPIPE means overrun */
      		fprintf(stderr, "overrun occurred\n");
      		snd_pcm_prepare(mp->handle);
    	} 
		else if (rc < 0) {
      		fprintf(stderr,
              "error from read: %s\n",
              snd_strerror(rc));
    	} 
		else if (rc != BUFF_SIZE) {
      		fprintf(stderr, "short read, read %d frames\n", rc);
    	}

   		rc = write(fileno(mp->file), (char *)mp->buffer, BUFF_SIZE * sizeof(short));

		if (rc != mp->size);
      		//fprintf(stderr, "short write: wrote %d bytes\n", rc);
}

void drainMic(struct micParams *mp) {
	int rc; 
	
	//Stops mic from recording and gathers remaining data. 
	if ((rc = snd_pcm_drain(mp->handle)) != 0) {
		fprintf(stderr, "Error draining PCM: %d\n", rc);  
		
		exit(1);  
	}

	while ((rc = snd_pcm_readi(mp->handle, (char *)mp->buffer, BUFF_SIZE)) > 0)
		write(fileno(mp->file), (char *)mp->buffer, BUFF_SIZE * sizeof(short)); 
}

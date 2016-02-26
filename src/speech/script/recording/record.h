#ifndef _RECORD_H
#define _RECORD_H

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>
#include <unistd.h>

typedef short int int16;

//Structure for microphone parameters. 
struct micParams {
 	int size;
  	snd_pcm_t *handle;
  	snd_pcm_hw_params_t *params;
  	unsigned int val;
  	int dir;
  	snd_pcm_uframes_t frames;
  	int16 *buffer;
	FILE* file; 
};

//Initializes the microphone for recording. 
int initMic(struct micParams *mp, char *dev_name, int num_channels); 

void initMics();

//Cleans up mic resources. 
void closeMic();

void closeMics(); 

void recordFromMic(struct micParams *mp); 

void drainMic(struct micParams *mp); 

/*Records mono audio at 1600Hz, little endian 
  and uses sphinx directly to recognize it. 
  The recognized utterance is written to a file. */
int record1600Hz(); 

void startRecord();

void stopRecord(); 

void interruptRecord(); 

#endif

#ifndef _RECORD_H
#define _RECORD_H

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>
#include <pocketsphinx.h>
#include <pthread.h>

//Structure for microphone parameters. 
struct micParams {
 	int size;
  	snd_pcm_t *handle;
  	snd_pcm_hw_params_t *params;
  	unsigned int val;
  	int dir;
  	snd_pcm_uframes_t frames;
  	int16 *buffer;
};

//Initializes the microphone for recording. 
int initMic(); 

//Cleans up mic resources. 
void closeMic(); 

/*Records mono audio at 1600Hz, little endian and
  outputs it to a file fout. */
int record1600Hz(const char *fout); 

/*Records mono audio at 1600Hz, little endian 
  and uses sphinx directly to recognize it. 
  The recognized utterance is written to a file. */
int record1600Hz_s(ps_decoder_t *ps); 

void startRecord();

void stopRecord(); 

#endif

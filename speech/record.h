#ifndef _RECORD_H
#define _RECORD_H

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>

/*Records mono audio at 1600Hz, little endian and
  outputs it to a file fout. */
int record1600Hz(const char *fout); 

void startRecord();

void stopRecord(); 

#endif

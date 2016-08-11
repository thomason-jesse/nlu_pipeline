/* -*- c-basic-offset:4; indent-tabs-mode: nil -*- */

/* ====================================================================
 * Copyright (c) 1999-2008 Carnegie Mellon University.  All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer. 
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY CARNEGIE MELLON UNIVERSITY ``AS IS'' AND 
 * ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY
 * NOR ITS EMPLOYEES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ====================================================================
 *
 */

#include "sphinx.h"

FILE* results = 0; 

//Globals

//Decoder for asr. 
ps_decoder_t* ps = 0;

void sphinx_init(const char *ac, const char* lm, const char* dict) {
	cmd_ln_t *config = 0;
	
	//TODO Check for and return errors. 
	config = cmd_ln_init(NULL, ps_args(), TRUE,
				"-hmm", ac,
				"-lm", lm,
				"-dict", dict,
				"-fwdtree", "yes",
				"-fwdflat", "yes",
				"-bestpath", "yes",
				"-input_endian", "little",
				"-samprate", "16000", NULL);

	ps = ps_init(config);
}

void sphinx_close() {
	if (ps)
		ps_free(ps);	
}

int sphinx_n_best(const char *phrase, const char *recording_path, const char* nbest_file_path, int n) {
	ps_nbest_t *nbest;
	char const *hyp;
	int32 score, log, defErr;

	//Opens recording. 
	FILE *raw_file = fopen(recording_path, "rb");

	//Opens nbest file.
	FILE *nbest_file = fopen(nbest_file_path, "a"); 

	//Decoder must be initialized prior to calling this function. 
	if (!ps) {
		printf("Decoder not initialized! Call sphinx_init prior to using this function."); 
		
		return -1; 
	}

	//Checks for IO errors. 
	if (!nbest_file) {
		printf("Problem opening nbest file! Tried to open in append mode, did you create file?"); 

		return -1; 
	}

	if (!raw_file) {
		printf("Error opening recording file!"); 
		
		return -1; 
	}

	//Decodes phrase and gets n-best hypotheses.  
	ps_decode_raw(ps, raw_file, -1);
	nbest = ps_nbest(ps);

	//Writes pound to delimit new phrase as well as its ground truth. 
	fprintf(nbest_file, "#%s\n", phrase);	

	int num = 0; 

	//Gets n hypotheses (or as many as possible if there are less than n available). 
	while ((num < n) && nbest && (nbest = ps_nbest_next(nbest))) {	
		//Gets next hypothesis and writes it.  
		hyp = ps_nbest_hyp(nbest, &score);
		fprintf(nbest_file, "%s;%d\n", hyp, score); 

		num++; 
	}

	//Frees nbest memory.  
	if (nbest)
	    ps_nbest_free(nbest);	

	//Closes files. 
	fclose(nbest_file); 
	fclose(raw_file); 

	return 0; 
}

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

ps_decoder_t* sphinx_init(const char *ac, const char* lm, const char* dict) {
	ps_decoder_t *ps = 0;
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

	/*
	if (config)
		cmd_ln_free_r(config);

	return ps; */

	return ps;  
}

void sphinx_close(ps_decoder_t *ps) {
	if (ps)
		ps_free(ps);	
}

int sphinx_n_best_f(const char *recording, const char *recordings_dir, const char* results_dir, int n, ps_decoder_t* ps) {
	ps_nbest_t *nbest;
	FILE *rawfh;
	char const *hyp;
	int32 score, log, defErr;

	if (!results){
		printf("Error creating results file!");

		return -1; 
	}

	//Opens recording. 
	char recording_name[300];
	strcpy(recording_name, recordings_dir); 
	strcat(recording_name, recording);
	strcat(recording_name, ".raw\0"); 
	rawfh = fopen(recording_name, "rb");

	if (!rawfh){
		printf("Problem opening input file!\n"); 

		return -1; 
	}

	//Opens nbeset file.
	char nbest_file_name[300];
	strcpy(nbest_file_name, results_dir); 
	strcat(nbest_file_name, recording); 
	strcat(nbest_file_name, ".nbest");
	FILE* nbest_file = fopen(nbest_file_name, "w"); 

	if (!nbest_file) {
		printf("Problem opening nbest file!"); 

		return -1; 
	}

	//Decodes phrase. 
	ps_decode_raw(ps, rawfh, -1);
	fclose(rawfh); 

	//Gets score for best result. 
	hyp = ps_get_hyp(ps, &score);
	int32 prob = ps_get_prob(ps);
	float conf = logmath_exp(ps_get_logmath(ps), prob); 

	//Writes recognition result. 
	fprintf(results, "%s\n", hyp); 

	nbest = ps_nbest(ps);
	
	int num = 1; 

	while ((num < n) && nbest && (nbest = ps_nbest_next(nbest))) {
		num++; 
		
		//Gets next hypothesis. 
		hyp = ps_nbest_hyp(nbest, &score);
		fprintf(nbest_file, "%s (%d)\n", hyp, score); 

		/* GETS individual word scores */
		/*
		ps_seg_t *seg;
		for (seg = ps_nbest_seg(nbest, &score); seg;
		     seg = ps_seg_next(seg)) {
			char const *word;
			int sf, ef;

			word = ps_seg_word(seg);
			ps_seg_frames(seg, &sf, &ef);
			printf("%s %d %d\n", word, sf, ef);
		}
		*/
	}

	if (nbest)
	    ps_nbest_free(nbest);	

	//Closes files. 
	fclose(nbest_file); 

	return 0; 
}

int main(int argc, char* argv[]) {
	ps_decoder_t* ps = sphinx_init(argv[1], argv[2], argv[3]);
	
	//Gets id file. 
	char ids_file_name [300];
	strcpy(ids_file_name, argv[4]);
	strcat(ids_file_name, argv[5]);
	FILE* ids_file = fopen(ids_file_name, "r"); 
	
	//Opens result file. 
	char results_name [300];
	strcpy(results_name, argv[4]);
	strcat(results_name, "results.txt"); 
	results = fopen(results_name, "w");

	if (!results) {
		printf("Problem opening results file!"); 

		return -1; 
	}

	char line[300]; 
	
	while (fgets(line, 300, ids_file)) {
		int index = strlen(line) - 1;

		printf("IN FGETS"); 

		if (index > 0) {
			printf("IN INDEX"); 
			line[strlen(line) - 1] = '\0';

			if (sphinx_n_best_f(line, argv[4], argv[6], 1000, ps) == -1) {
				printf("Sphinx encountered a recognition error!");

				return -1; 
			}
		}
	}

	//Closes resources. . 
	fclose(results); 
	sphinx_close(ps);

	return 0; 
}

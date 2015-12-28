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
#include "record.h"

static int interrupted = 0; 
static ps_decoder_t *ps = 0;
static cmd_ln_t *config = 0;

void sphinx_interrupt() {
	interrupted = 1; 
}

void sphinx_init() {
	//TODO Check for and return errors. 
	config = cmd_ln_init(NULL, ps_args(), TRUE,
				"-hmm", "./src/nlu_pipeline/src/speech/sphinx_resources/en-us/en-us",
				"-lm", "./src/nlu_pipeline/src/speech/sphinx_resources/en-us/fifty_fifty.lm.bin",
				"-dict", "./src/nlu_pipeline/src/speech/sphinx_resources/en-us/cmudict-en-us.dict",
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
	
	if (config)
		cmd_ln_free_r(config);
}

int sphinx_n_best_f(const char *file, int n) {
	ps_nbest_t *nbest;
	FILE *rawfh;
	char const *hyp;
	int32 score, log, defErr;
	int backup, temp; 

	backup = dup(2);
	temp = open("/dev/null", O_WRONLY);
	dup2(temp, 2); 
	close(temp);

	//File to write results into. 
	FILE *results = fopen("results.txt", "w");

	if (!results){
		printf("Error creating results file!");

		return 0; 
	}

	rawfh = fopen(file, "rb");

	if (!rawfh){
		printf("Problem opening input file!"); 

		return 0; 
	}

	ps_decode_raw(ps, rawfh, -1);
	fclose(rawfh);
	
	hyp = ps_get_hyp(ps, &score);
	int32 prob = ps_get_prob(ps);
	float conf = logmath_exp(ps_get_logmath(ps), prob); 

	fprintf(results, "%s:%d:%d:%g\n", hyp, score, prob, conf);

	nbest = ps_nbest(ps, 0, -1, NULL, NULL);
	
	int num = 1; 

	while ((num < n) && nbest && (nbest = ps_nbest_next(nbest))) {
		ps_seg_t *seg;
		hyp = ps_nbest_hyp(nbest, &score);

		fprintf(results, "NBEST %d: %s (%d)\n", n, hyp, score); 

		/* GETS individual word scores */
		/*
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

	//Closes results file. 
	fclose(results); 

	dup2(backup, 2); 
	close(backup); 

	return 0; 
}

int sphinx_n_best_m(int n) {
	ps_nbest_t *nbest;
	char const *hyp;
	int32 score, log, defErr;
	int backup, temp; 

	backup = dup(2);
	temp = open("/dev/null", O_WRONLY);
	dup2(temp, 2); 
	close(temp);

	//File to write results into. 
	FILE *results = fopen("./src/nlu_pipeline/src/speech/data/recording/results.txt", "w");

	if (!results){
		printf("Error creating results file!");

		return 0; 
	} 

	//Records and processes an utterance. 
	record1600Hz_s(ps); 

	//FILE *rawfh = fopen("voice.raw", "rb"); 
	//ps_decode_raw(ps, rawfh, -1); 
	//fclose(rawfh); 

	if (!interrupted) {
		//hyp = ps_get_hyp(ps, &score);
		//int32 prob = ps_get_prob(ps);
		//float conf = logmath_exp(ps_get_logmath(ps), prob); 

		nbest = ps_nbest(ps, 0, -1, NULL, NULL);
	
		int num = 1; 

		while ((num <= n) && nbest && (nbest = ps_nbest_next(nbest))) {
			ps_seg_t *seg;
			hyp = ps_nbest_hyp(nbest, &score);

			fprintf(results, "Result %d: %s\n", num, hyp); 

			/* GETS individual word scores */
			/*
			for (seg = ps_nbest_seg(nbest, &score); seg;
		    	seg = ps_seg_next(seg)) {
				char const *word;
				int sf, ef;

				word = ps_seg_word(seg);
				ps_seg_frames(seg, &sf, &ef);
				printf("%s %d %d\n", word, sf, ef);
			}
			*/

			num++;
		}
	}

	if (nbest)
	    ps_nbest_free(nbest);	

	//Closes results file. 
	fclose(results); 

	dup2(backup, 2); 
	close(backup); 

	return 0; 
}

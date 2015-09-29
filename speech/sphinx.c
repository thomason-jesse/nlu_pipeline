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

int sphinx_n_best(const char *file, int n) {
	ps_decoder_t *ps;
	ps_nbest_t *nbest;
	cmd_ln_t *config;
	FILE *rawfh;
	char const *hyp;
	int32 score, log, defErr;

	//File to write results into. 
	FILE *results = fopen("results.txt", "w");

	if (!results){
		printf("Error creating results file!");

		return 0; 
	}

	//Creates a log file. 
	log = open("log.txt", O_WRONLY);

	if (log < 0) {
		//printf("Could not open log file!\n"); 

		//return 0; 
	}

	//Clones stdout in order to change back later. 
	defErr = dup(2); 

	//Redirects stdout to log file. 
	if (dup2(log, 2) < 0) {
		//printf("Could not redirict stderr!\n");

		//return 0; 
	}

	close(2);

	config = cmd_ln_init(NULL, ps_args(), TRUE,
				"-hmm", MODELDIR "/en-us/en-us",
				"-lm", MODELDIR "/en-us/en-us.lm.bin",
				"-dict", MODELDIR "/en-us/cmudict-en-us.dict",
				"-fwdtree", "yes",
				"-fwdflat", "yes",
				"-bestpath", "yes",
				"-input_endian", "little",
				"-samprate", "16000", NULL);

	ps = ps_init(config);

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
	
	ps_free(ps);
	cmd_ln_free_r(config);

	//Closes results and log files. 
	fclose(results); 
	//close(log); 

	//Brings back stderr. 
	if (dup2(defErr, 2) < 0) {
		printf("Could not redirect stderr back!"); 

		return -1; 
	}
	//close(defErr); 

	return 0; 
}

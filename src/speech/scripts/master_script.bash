for time_limit in $( echo 5 10 100 ); do 

	for data_percent in $( echo 0.05 0.25 0.5 0.75 1.0 ); do 
		#Parser hyper parameters. 
		hyp1=2   		 		# max span of a multi-word expression to be considered during tokenization [2]
		hyp2=2  		 		# max number of new word senses that can be induced on a training example [2]
		hyp3=100    	 		# for tokenization of an utterance, max cky trees considered [100]
		hyp4=2  		 		# for unknown token, max syntax categories tried [2]
		hyp5=10  		 		# decides how many expansions to store per CKY cell [10]
		hyp6=${time_limit}		# Parsing budget per example during training. [10]
		hyp7=True		 		# Whether or not to use default token skipping behavior. [True]
		hyp8=${data_percent}	# Percentage of total training data to use. 

		#Hyperparameters denote naming convention for different files we need. 
		name="${hyp1}_${hyp2}_${hyp3}_${hyp4}_${hyp5}_${hyp6}_${hyp7}_${hyp8}"

		for fold in $( echo 0 1 2 3 4 5 6 7 8 9 ); do
			fold_path=../corpora/processed/10_folds_all_lengths_google_numbers/${fold}

			#Path to log for condor script and training procedure of first epoch. 
			log=${fold_path}/logs/training/${name}.log

			#Path to output for condor script and training procedure of first epoch. 
			output=${fold_path}/logs/training/${name}.out

			condor_submit -a "parser_params = ${name}" -a "log = ${log}" -a "output = ${output}" -a "fold_path = ${fold_path}"  submit_train.bash
		done
	done
done

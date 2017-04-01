starting_parser_path=$2



#python train.py new_parser condor_test_dir/parser_train.txt parser.cky ../resources/ont.txt ../resources/lex.txt 50.0 1.0 None ${name} 

for time_limit in $( echo 2 5 10 20 50 100 ); do 
	#Parser hyper parameters. 
	hyp1=2   		 		# max span of a multi-word expression to be considered during tokenization [2]
	hyp2=2  		 		# max number of new word senses that can be induced on a training example [2]
	hyp3=100    	 		# for tokenization of an utterance, max cky trees considered [100]
	hyp4=2  		 		# for unknown token, max syntax categories tried [2]
	hyp5=10  		 		# decides how many expansions to store per CKY cell [10]
	hyp6=${time_limit}		# Parsing budget per example during training. [10]
	hyp7=True		 		# Whether or not to use default token skipping behavior. [True]

	#Parser parameters in a string. 
	parser_params="${hyp1}_${hyp2}_${hyp3}_${hyp4}_${hyp5}_${hyp6}_${hyp7}"

	for data_percent in $( echo 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 ); do 
		#Hyperparameters denote naming convention for different files we need. 
		epoch=0
		name="${parser_params}_${data_percent}_${epoch}"

		for fold in $( echo 0 1 2 3 4 5 6 7 8 9 ); do
			fold_path=../corpora/processed/10_folds_all_lengths_google_numbers/${fold}
			log_path=${fold_path}/logs/training/${name}.log

			echo ${log_path}

			#condor_submit -a "data_percent = ${data_percent}" -a "starting_parser_path = ${starting_parser_path}" -a "log = ${log}" -a "output = ${output}" -a "error = ${output}" submit_condor.bash

		done
	done
done

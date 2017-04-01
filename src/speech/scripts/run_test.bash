#Argument list. 
parser_params=$1 #Parser hyperparameters in a string.  
fold_path=$2	 #Path to the fold for the current experiment.  

#Keep evaluating parsers until we kill the job or it dies. 
epoch=0

while true; do
	#Name denoted by parser hyperparameters and epoch. 
	name="${parser_params}_${epoch}"
	parser_path=${fold_path}/models/${name}.cky	

	#If parser exists, then evaluate it. 
	if [ -f ${parser_path} ]; then
   		echo "Testing parser at ${parser_path}, epoch ${epoch}"

		python experiments.py parse_score_google ${fold_path}/experiments/asr/test_files/test0.txt ${fold_path}/experiments/asr/result_files/${name}.test0 ${parser_path} ../corpora/raw/google_recognized_with_names/

	#Otherwise, exit with an error code for Condor. 
   	else
    	echo "Finished testing all parsers in fold with given hyperparameters."
		exit 0
	fi

	#Update epoch. 
	epoch=$((${epoch} + 1))
done

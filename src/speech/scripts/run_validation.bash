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
   		echo "Evaluating parser at ${parser_path}, epoch ${epoch}"

		python experiments.py parse_ground_truth ${fold_path}/experiments/asr/test_files/validation0.txt ${fold_path}/experiments/asr/result_files/${name}.validation0 ${parser_path}

	#Otherwise, exit with an error code for Condor. 
   	else
    	echo "Finished evaluating all parsers in fold with given hyperparameters."
		exit 0
	fi

	#Update epoch. 
	epoch=$((${epoch} + 1))
done

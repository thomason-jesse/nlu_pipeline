#Argument list. 
parser_params=$1 #Parser hyperparameters in a string.  
fold_path=$2	 #Path to the fold for the current experiment.  

#Keep training parsers until we kill the job or it dies. 
starting_parser="None"
epoch=0

while true; do
	#Name denoted by parser hyperparameters and epoch. 
	name="${parser_params}_${epoch}"
	parser_path=${fold_path}/models/${name}.cky	

	python train.py new_parser ${fold_path}/parser_training/parser_train.txt ${parser_path} ../resources/ont.txt ../resources/lex.txt 50.0 ${starting_parser} ${parser_params}

	#If parser exists, then evaluate it. 
	if [ -f ${parser_path} ]; then
   		echo "Trained parser at ${parser_path}, epoch ${epoch}"

		#Now submit job to evaluate performance on validation set.  
		condor_submit -a "fold_path = ${fold_path}" -a "name = ${name}" -a "log = ${fold_path}/logs/validation/${name}.log" -a "output = ${fold_path}/logs/validation/${name}.out" submit_validation.bash

		#And finally submit job to evaluate performance on 
		condor_submit -a "fold_path = ${fold_path}" -a "name = ${name}" -a "log = ${fold_path}/logs/test/${name}.log" -a "output = ${fold_path}/logs/test/${name}.out" submit_test.bash
		
		#Update starting parser and epoch. 
		starting_parser=${parser_path}
		epoch=$((${epoch} + 1))

	#Otherwise, exit with an error code for Condor. 
   	else
    	echo "Failed to save parser at ${parser_path}!"
		exit 1
	fi
done




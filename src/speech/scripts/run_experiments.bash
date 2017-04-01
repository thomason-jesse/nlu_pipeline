#Argument list. 
data_percent=$1 #Percentage of training data to use per epoch of training. 
starting_parser_path=$2 #Path to the parser with which to start training procedure. 

#First train a parser. 

#python train.py new_parser condor_test_dir/parser_train.txt condor_test_dir/parser_0.cky ../resources/ont.txt ../resources/lex.txt 50.0 ${data_percent} ${starting_parser_path}

#python experiments.py parse_ground_truth condor_test_dir/test.txt condor_test_dir/gt_results.txt condor_test_dir/parser_0.cky

python experiments.py parse_score_google condor_test_dir/test.txt condor_test_dir/google_results.txt condor_test_dir/parser_0.cky ../corpora/raw/google_recognized_with_names/

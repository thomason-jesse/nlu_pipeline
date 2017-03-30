#First train a parser. 
#python train.py new_parser ../corpora/processed/8_folds_all_lengths_google_numbers/0/parser_training/parser_train.txt test_parser.cky ../resources/ont.txt ../resources/lex.txt 50.0

python train.py new_parser condor_test_dir/parser_train.txt condor_test_dir/test_parser.cky ../resources/ont.txt ../resources/lex.txt 50.0

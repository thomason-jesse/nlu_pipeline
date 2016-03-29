universe = vanilla

Initialdir = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src
Executable = /usr/bin/python

+Group   = "GRAD"
+Project = "AI_ROBOTICS"
+ProjectDescription = "Training semantic parser using template generated examples"

Log = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/condor_log/parser_training_1000_easy.log
Output = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/condor_log/parser_training_1000_easy.out
Error = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/condor_log/parser_training_1000_easy.err

Notification = complete
Notify_user = aish@cs.utexas.edu

Arguments = ParserTrainer.py /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/resources/fake_to_real/ont.txt /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/resources/fake_to_real/lex.txt /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/tests/templated_examples/1000.txt

Queue 1


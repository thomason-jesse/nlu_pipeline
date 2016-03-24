universe = vanilla

Initialdir = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src
Executable = /usr/bin/python

+Group   = "GRAD"
+Project = "AI_ROBOTICS"
+ProjectDescription = "Training semantic parser using template generated examples"

Log = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/condor_log/parser_training_100_easy.log
Output = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/condor_log/parser_training_100_easy.out
Error = /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/condor_log/parser_training_100_easy.err

Notification = complete
Notify_user = aish@cs.utexas.edu

Arguments = ParserTrainer100.py /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/resources/fake_to_real/ont.txt /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/resources/fake_to_real/lex.txt /u/aish/Documents/Research/Code/catkin_ws/src/nlu_pipeline/src/resources/fake_to_real/utterance_sem_forms.txt

Queue 1


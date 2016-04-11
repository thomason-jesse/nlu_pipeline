universe = vanilla
environment = /u/rcorona/nlu_pipeline/src/speech/experiments

STANFORD_PATH=/u/rcorona/388/HW3/stanford-parser

Initialdir = /u/rcorona/nlu_pipeline/src/speech/experiments
Executable = $(SRILM_DIR)/ngram

+Group = "UNDER"
+Project = "INSTRUCTIONAL"
+ProjectDescription = "CS 379H Project"

Log = $(NAME).log

Notification = complete
Notify_user = rcorona@utexas.edu

Arguments = -lm $(IN_LM) -mix-lm $(GEN_LM) -lambda $(w) -write-lm $(INT_LM) 

Output = $(NAME).out
Error = $(NAME).err
Queue 1

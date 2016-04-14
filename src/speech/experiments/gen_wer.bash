requirements = InMastodon
universe = vanilla
environment = /scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments

Initialdir = /scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/
Executable = $(BIN)/word_align.pl

+Group = "UNDER"
+Project = "INSTRUCTIONAL"
+ProjectDescription = "CS 379H Project"

Log = $(LOGS).log

Notification = complete
Notify_user = rcorona@utexas.edu

Arguments = $(TRANSCRIPT) $(RESULTS) 

Output = $(LOGS).out
Error = $(LOGS).err
Queue 1


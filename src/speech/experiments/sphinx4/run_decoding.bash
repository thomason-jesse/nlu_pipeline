universe = java
requirements = InMastodon
executable = Experiments.class
jar_files = sphinx4-core.jar,sphinx4-data.jar

LOGS=/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/condor/logs/decodings

+Group = "UNDER"
+Project = "INSTRUCTIONAL"
+ProjectDescription = "CS 379H Project"

Log = $(LOGS)/$(NAME).log

Notification = complete
Notify_user = rcorona@utexas.edu

Arguments = Experiments $(AC_MODEL) $(DICT) $(LM) $(RESULTS_DIR) $(RECORDINGS_DIR) $(IDS_FILE) $(NAME)

Output = $(LOGS)/$(NAME).out
Error = $(LOGS)/$(NAME).err
Queue 1
